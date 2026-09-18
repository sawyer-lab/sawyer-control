#!/usr/bin/env python3
"""Compare sensor configurations against the same physical motion.

`ft_sandbox.py` captures each configuration whenever it gets to it, so a
difference between trials mixes the setting with whatever the sensor happened to
be feeling at the time. That is fine for noise on an untouched sensor and
useless for anything you are actually pushing on.

This tool removes the confound by repeating the motion: it configures the box,
waits for you, captures, and does it again for the next configuration. The
excitation is repeatable rather than identical - your hand and the arm are not
perfect - but it is the same motion through the same poses, which is what makes
the traces worth overlaying.

The motion is the CSV trajectory, played through the arm at a fixed rate, so
every trial sees the same poses at the same speeds, and the pass for one
configuration can be read against the pass for another. Only sensor data is
recorded - the joints are the excitation, not the measurement.

Start the bridge with FT_SENSOR_IP=disabled: it keeps commanding the robot and
stops touching the sensor, so this script owns the RDT stream outright.

Only device globals are varied - RDT rate, low-pass filter, bias vector. The
active configuration slot is never selected: the bridge caches that slot's
counts-per-unit factors at startup and would silently mis-scale every reading
until restarted. Slot parameters are `ft_sandbox.py`'s job.

Edit the block below and run it.
"""

from __future__ import annotations

import csv
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "bridge"))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from _rdt import AXES, stream  # noqa: E402
from robot_api.hardware.ft_config import FILTER_CUTOFF_HZ, NetBoxConfig  # noqa: E402
from sawyer_control import ControlMode, JointCommandSample, SawyerRobotClient  # noqa: E402


# ─────────────────────────────────────────────────────────────────────────────
# Edit from here.
# ─────────────────────────────────────────────────────────────────────────────

HOST = "192.168.1.11"
BRIDGE = "127.0.0.1:50051"

TRAJECTORY_CSV = "/home/fausto/Projects/sawyer-operations/data/replay_results.csv"
RATE_HZ = 100.0          # command rate; the CSV is downsampled to it
APPROACH_TIMEOUT_S = 30.0
SETTLE_S = 1.0           # stand still before each pass
BETWEEN_S = 1.0          # stand still after one, before the next

# One pass of the trajectory each. Keys: label, rate, filter, bias.
#   bias=True   the vector the box had when this script started
#   bias=False  cleared
#
# Filter codes are not monotonic: 1-8 descend from 838 Hz to 5 Hz, and 9-12 are
# *higher* cutoffs than 1. Achievable rates are integer divisors of 7000 and the
# box rounds up, so the label reports what it actually accepted.
#
# The sweep below asks three questions in one run. At full rate, what does the
# filter cost you (rows 1-5)? At the rate you actually use, is the filter doing
# anything the rate is not (rows 6-7)? And what does a low rate with no filter
# alias into the band (rows 8-9)?
TRIALS = [
    dict(rate=7000, filter=0),       # reference: everything the box can see
    dict(rate=7000, filter=1),       # 838 Hz
    dict(rate=7000, filter=3),       # 152 Hz
    dict(rate=7000, filter=5),       # 35 Hz
    dict(rate=7000, filter=8),       # 5 Hz - well inside the motion
    dict(rate=1000, filter=0),
    dict(rate=1000, filter=3),       # 152 Hz - the live setting
    dict(rate=100,  filter=0),       # aliases everything above 50 Hz
    dict(rate=100,  filter=6),       # 18 Hz - matched to the 50 Hz Nyquist
]

RESTORE = True
# One figure per trial is written as SAVE with an index and the trial label
# inserted, e.g. ft_compare_03_1000hz_152hz.png. SHOW opens all nine at once.
SHOW = False
SAVE = "ft_compare.png"

# ─────────────────────────────────────────────────────────────────────────────


def load_trajectory(path, rate_hz):
    """Read `t,q1..q7` and downsample to `rate_hz`.

    The solver writes at 1 kHz; the arm is commanded at 100, so every tenth row
    is taken. The factor is derived from the file's own timestep rather than
    assumed, so a differently sampled CSV still plays at the right speed.
    """
    times, rows = [], []
    with open(path, newline="") as handle:
        for line in csv.reader(handle):
            if len(line) < 8:
                continue
            try:
                values = [float(v) for v in line[:8]]
            except ValueError:
                continue                       # header
            times.append(values[0])
            rows.append(values[1:8])
    if len(rows) < 2:
        raise SystemExit(f"{path}: need at least two rows of t,q1..q7")
    source_hz = 1.0 / (times[1] - times[0])
    factor = max(1, round(source_hz / rate_hz))
    played = rows[::factor]
    print(f"{Path(path).name}: {len(rows)} rows at {source_hz:.0f} Hz -> "
          f"every {factor} = {len(played)} samples at {source_hz / factor:.0f} Hz, "
          f"{len(played) / (source_hz / factor):.2f} s per pass")
    return played


class Capture(threading.Thread):
    """Reads RDT in its own thread so the command loop keeps its schedule."""

    def __init__(self, host, scale_f, scale_t):
        super().__init__(daemon=True)
        self.host, self.scale_f, self.scale_t = host, scale_f, scale_t
        self.rows, self.times = [], []
        self._stop = threading.Event()

    def run(self):
        generator = stream(self.host, 3600.0, self.scale_f, self.scale_t)
        for elapsed, row in generator:
            self.rows.append(row)
            self.times.append(elapsed)
            if self._stop.is_set():
                generator.close()
                return

    def stop(self):
        self._stop.set()
        self.join(timeout=3.0)


def play(robot, rows, rate_hz):
    """Command each sample against an absolute schedule, so the error in one
    period does not accumulate over the whole pass. Returns worst lateness."""
    period_ns = round(1_000_000_000 / rate_hz)
    start_ns = time.monotonic_ns()
    worst_ms = 0.0
    for index, position in enumerate(rows):
        deadline_ns = start_ns + index * period_ns
        delay_s = (deadline_ns - time.monotonic_ns()) / 1e9
        if delay_s > 0:
            time.sleep(delay_s)
        worst_ms = max(worst_ms, (time.monotonic_ns() - deadline_ns) / 1e6)
        robot.command(JointCommandSample(position=position), ControlMode.POSITION)
    return worst_ms


def describe(trial):
    if trial.get("label"):
        return trial["label"]
    parts = [f"{trial['rate']} Hz"] if "rate" in trial else []
    if "filter" in trial:
        cutoff = FILTER_CUTOFF_HZ.get(trial["filter"])
        parts.append("no filter" if cutoff is None else f"{cutoff} Hz LPF")
    if "bias" in trial:
        parts.append("bias on" if trial["bias"] else "bias off")
    return ", ".join(parts)


def apply_trial(box, trial, saved_bias):
    """Returns the rate the box accepted; it rounds up to a divisor of 7000."""
    if "bias" in trial:
        box.set_bias_vector(saved_bias if trial["bias"] else [0] * 6)
    if "filter" in trial:
        box.set_filter(trial["filter"])
    return box.set_rate_hz(trial["rate"]) if "rate" in trial else box.communications()["rate_hz"]


def restore(box, settings, comms):
    box.set_rate_hz(comms["rate_hz"])
    box.set_filter(settings["filter_code"])
    if settings["bias_vector"]:
        box.set_bias_vector(settings["bias_vector"])


def column(rows, index):
    return [r[index] for r in rows]


def sigma(values):
    average = sum(values) / len(values)
    return (sum((v - average) ** 2 for v in values) / len(values)) ** 0.5


def run(trials=None, host=None, csv_path=None, rate_hz=None, restore_after=None):
    """Play the trajectory once per trial and capture the sensor through it."""
    trials = TRIALS if trials is None else trials
    host = HOST if host is None else host
    csv_path = TRAJECTORY_CSV if csv_path is None else csv_path
    rate_hz = RATE_HZ if rate_hz is None else rate_hz
    restore_after = RESTORE if restore_after is None else restore_after

    trajectory = load_trajectory(csv_path, rate_hz)
    print(f"{len(trials)} trials, so {len(trials)} passes of the same motion.\n")
    print("THE ARM WILL MOVE: to the first sample, then through the trajectory, "
          "once per trial.")
    input("Press Enter to start, Ctrl-C to abort. ")

    box = NetBoxConfig(host)
    config = box.configuration()
    scale_f, scale_t = config["counts_per_force"], config["counts_per_torque"]
    saved_settings, saved_comms = box.settings(), box.communications()
    saved_bias = saved_settings["bias_vector"] or [0] * 6
    print(f"\nNet F/T at {host}: active slot {config['active_slot']}, "
          f"units {config['force_unit']}/{config['torque_unit']}, bias {saved_bias}")

    robot = SawyerRobotClient.connect(BRIDGE)
    state = robot.get_state()
    if not state.enabled:
        robot.close()
        raise SystemExit("the robot is not enabled; run "
                         "`python scripts/demo/robot.py enable` first")

    results = []
    try:
        for index, trial in enumerate(trials, 1):
            label = describe(trial)
            effective = apply_trial(box, trial, saved_bias)
            print(f"\n  [{index}/{len(trials)}] {label}")
            robot.move_to(trajectory[0], APPROACH_TIMEOUT_S)
            time.sleep(SETTLE_S)

            capture = Capture(host, scale_f, scale_t)
            capture.start()
            time.sleep(0.2)                    # let the stream come up
            worst_ms = play(robot, trajectory, rate_hz)
            capture.stop()
            time.sleep(BETWEEN_S)          # let the arm and the sensor settle

            if not capture.rows:
                print("      no sensor data received")
                continue
            noises = [sigma(column(capture.rows, i)) for i in range(6)]
            measured = len(capture.rows) / capture.times[-1] if capture.times[-1] else 0.0
            print(f"      {len(capture.rows)} samples at {measured:.0f} Hz, "
                  f"command loop worst lateness {worst_ms:.1f} ms, "
                  f"Fz sigma {noises[2]:.4f} {config['force_unit']}")
            results.append({"label": label, "rows": capture.rows,
                            "times": capture.times, "sigma": noises,
                            "rate": effective, "worst_lateness_ms": worst_ms,
                            "force_unit": config["force_unit"],
                            "torque_unit": config["torque_unit"]})
    finally:
        robot.close()
        if restore_after:
            restore(box, saved_settings, saved_comms)
            print(f"\nrestored: rate {box.communications()['rate_hz']} Hz, "
                  f"filter code {box.settings()['filter_code']}, "
                  f"bias {box.settings()['bias_vector']}")
        else:
            print("\nRESTORE is off: sensor left as the last trial set it")
    return results


def report(results):
    if not results:
        return
    print(f"\nsigma per axis, force in {results[0]['force_unit']}, "
          f"torque in {results[0]['torque_unit']}")
    print(f"  {'trial':<28}" + "".join(f"{a:>10}" for a in AXES))
    for row in results:
        print(f"  {row['label']:<28}" + "".join(f"{v:>10.4f}" for v in row["sigma"]))


def plot(results, save=None, show=None):
    """One figure per trial, all six axes, written as separate files.

    A single overlaid figure sounds appealing and is unreadable: nine traces of
    a 7.5 s motion sit on top of each other, and the axes that matter are not
    always Fz. One page per configuration, six panels each, is what you actually
    read afterwards.
    """
    if not results:
        return
    save = SAVE if save is None else save
    show = SHOW if show is None else show

    import numpy as np
    import matplotlib
    if save and not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    force_unit = results[0]["force_unit"]
    torque_unit = results[0]["torque_unit"]
    units = [force_unit] * 3 + [torque_unit] * 3
    stem = Path(save) if save else None
    written = []

    for index, result in enumerate(results, 1):
        figure, panels = plt.subplots(3, 2, figsize=(13, 8), sharex=True)
        times = result["times"]
        for axis_index, (name, unit) in enumerate(zip(AXES, units)):
            panel = panels[axis_index % 3][axis_index // 3]
            values = column(result["rows"], axis_index)
            panel.plot(times, values, linewidth=0.6)
            panel.axhline(np.mean(values), color="tab:red", linewidth=0.8, linestyle="--")
            panel.set_title(f"{name}   sigma {result['sigma'][axis_index]:.4f} {unit}",
                            fontsize=9)
            panel.set_ylabel(unit, fontsize=8)
            panel.grid(alpha=0.3)
        for panel in panels[2]:
            panel.set_xlabel("time (s)", fontsize=8)

        figure.suptitle(f"{result['label']}   -   {len(result['rows'])} samples, "
                        f"command loop worst lateness "
                        f"{result.get('worst_lateness_ms', 0):.1f} ms", fontsize=11)
        figure.tight_layout(rect=(0, 0, 1, 0.96))

        if stem is not None:
            slug = (result["label"].replace(" Hz", "hz").replace(" LPF", "")
                    .replace(", ", "_").replace(" ", "-"))
            path = stem.with_name(f"{stem.stem}_{index:02d}_{slug}{stem.suffix}")
            figure.savefig(path, dpi=130)
            written.append(path)

    if written:
        print(f"\nwrote {len(written)} figures:")
        for path in written:
            print(f"  {path}")
    if show:
        plt.show()


if __name__ == "__main__":
    results = run()
    report(results)
    plot(results)
