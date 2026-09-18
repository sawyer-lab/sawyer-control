#!/usr/bin/env python3
"""Scratch-slot sandbox: change sensor settings from code, measure, plot.

The other demo tools answer fixed questions. This one is meant to be edited.
Change the block below and run it, or import `run` and drive it from your own
script or a REPL:

    from ft_sandbox import run, report, plot
    results = run([dict(rate=1000, filter=0), dict(rate=1000, filter=3)])
    report(results); plot(results)

Two kinds of parameter exist on the Net Box and they behave differently:

  globals   device-wide, shared by every configuration slot: RDT output rate,
            low-pass filter, software bias vector, peak logging. TRIALS and
            GLOBALS change these.
  slot      per-configuration: calibration, units, tool transform, name. Only
            in effect while that slot is active. SLOT_PARAMS changes these.

Everything touched is read first and written back on exit, including after a
failure or Ctrl-C, so a run leaves the box as it found it. Set RESTORE = False
to make the changes stick instead.

Slot 0 is never written. It holds the working configuration and this tool
refuses it outright; the default scratch slot is 8.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "bridge"))

from _rdt import AXES, capture  # noqa: E402
from robot_api.hardware.ft_config import FILTER_CUTOFF_HZ, NetBoxConfig  # noqa: E402


# ─────────────────────────────────────────────────────────────────────────────
# Edit from here.
# ─────────────────────────────────────────────────────────────────────────────

HOST = "192.168.1.11"
SLOT = 8                  # scratch slot. Never 0.
SECONDS = 2.0             # capture length per trial
RESTORE = True            # put every setting back on the way out
SHOW = True               # plt.show() at the end; set a path in SAVE instead
SAVE = None               # e.g. "sandbox.png" -> also writes sandbox_summary.png

# Per-slot parameters, applied once before any trial. Anything omitted is left
# alone. Units take the manual's menu name ("N", "Nm", "mm") or a code.
SLOT_PARAMS = dict(
    # name="Sandbox",
    # calibration=0,
    # force_unit="N",
    # torque_unit="Nm",
    # distance_unit="mm",
    # angle_unit="degrees",
    # tool=(0, 0, 35, 0, 0, 0),     # dx dy dz rx ry rz, in distance_unit
    # user_a="", user_b="",
)

# Device globals held across every trial.
GLOBALS = dict(
    # peak_logging=False,
    # rdt=True,
    # monitor_conditions=False,
)

# One capture each. Keys: label, rate, filter, bias, buffer. Anything omitted is
# left as the previous trial left it.
#
#   bias=False  clear the software bias vector
#   bias=True   restore the vector the box had when this script started
#   bias=[...]  six explicit per-gage offsets
#
# The default set asks what the bias does and what rate and filter do on top of
# it: bias moves the mean and leaves the noise alone, the filter does the
# opposite, and a low rate with no filter aliases everything into the band.
TRIALS = [
    dict(bias=False, rate=1000, filter=0),
    dict(bias=True,  rate=1000, filter=0),
    dict(bias=True,  rate=1000, filter=3),    # 152 Hz LPF - the live setting
    dict(bias=True,  rate=100,  filter=0),
]

# ─────────────────────────────────────────────────────────────────────────────
# Below here is the machinery.
# ─────────────────────────────────────────────────────────────────────────────

WORKING_SLOT = 0
TRIAL_KEYS = {"label", "rate", "filter", "bias", "buffer"}
GLOBAL_KEYS = {"peak_logging", "rdt", "monitor_conditions"}
SLOT_KEYS = {"name", "calibration", "force_unit", "torque_unit",
             "distance_unit", "angle_unit", "user_a", "user_b", "tool"}


def _check(fields, allowed, what):
    """A typo in a dict key should fail, not be silently ignored."""
    unknown = set(fields) - allowed
    if unknown:
        raise ValueError(f"{what}: unknown {', '.join(sorted(unknown))}; "
                         f"known keys are {', '.join(sorted(allowed))}")


def describe(trial):
    if trial.get("label"):
        return trial["label"]
    parts = []
    if "rate" in trial:
        parts.append(f"{trial['rate']} Hz")
    if "filter" in trial:
        cutoff = FILTER_CUTOFF_HZ.get(trial["filter"])
        parts.append("no filter" if cutoff is None else f"{cutoff} Hz LPF")
    if "bias" in trial:
        bias = trial["bias"]
        parts.append("bias on" if bias is True else "bias off" if bias is False else "bias set")
    if "buffer" in trial:
        parts.append(f"{trial['buffer']} rec/pkt")
    return ", ".join(parts) or "as found"


# ── device state: capture, apply, restore ────────────────────────────────────

def snapshot(box, slot):
    """Everything this tool is capable of changing, so it can all go back."""
    return {"settings": box.settings(), "comms": box.communications(),
            "slot": box.configuration(slot)}


def restore(box, slot, saved):
    settings, comms, config = saved["settings"], saved["comms"], saved["slot"]
    box.set_rate_hz(comms["rate_hz"])
    box.set_buffer_records(comms["buffer_records"])
    box.set_rdt_enabled(comms["rdt_enabled"])
    box.set_filter(settings["filter_code"])
    box.set_peak_logging(settings["peak_logging"])
    box.set_monitor_conditions_enabled(settings["monitor_conditions_enabled"])
    if settings["bias_vector"]:
        box.set_bias_vector(settings["bias_vector"])
    box.write_configuration(slot, name=config["name"], calibration=config["calibration"],
                            force_unit=config["force_unit_code"],
                            torque_unit=config["torque_unit_code"],
                            distance_unit=config["tool_distance_unit_code"],
                            angle_unit=config["tool_angle_unit_code"],
                            user_field_a=config["user_field_a"],
                            user_field_b=config["user_field_b"])
    box.set_tool_transform(slot, *config["tool_transform"],
                           distance_unit=config["tool_distance_unit_code"],
                           angle_unit=config["tool_angle_unit_code"])
    box.select_configuration(settings["active_slot"])


def apply_slot(box, slot, fields):
    _check(fields, SLOT_KEYS, "SLOT_PARAMS")
    if not fields:
        return
    transform = fields.get("tool")
    if set(fields) - {"tool"}:
        box.write_configuration(
            slot, name=fields.get("name"), calibration=fields.get("calibration"),
            force_unit=fields.get("force_unit"), torque_unit=fields.get("torque_unit"),
            distance_unit=fields.get("distance_unit"), angle_unit=fields.get("angle_unit"),
            user_field_a=fields.get("user_a"), user_field_b=fields.get("user_b"))
    if transform is not None:
        if len(transform) != 6:
            raise ValueError("tool needs six values: dx, dy, dz, rx, ry, rz")
        box.set_tool_transform(slot, *transform,
                               distance_unit=fields.get("distance_unit"),
                               angle_unit=fields.get("angle_unit"))


def apply_globals(box, fields):
    _check(fields, GLOBAL_KEYS, "GLOBALS")
    if "peak_logging" in fields:
        box.set_peak_logging(fields["peak_logging"])
    if "rdt" in fields:
        box.set_rdt_enabled(fields["rdt"])
    if "monitor_conditions" in fields:
        box.set_monitor_conditions_enabled(fields["monitor_conditions"])


def apply_trial(box, trial, saved_bias):
    """Returns the rate the box actually accepted; it rounds up silently."""
    _check(trial, TRIAL_KEYS, "TRIALS")
    if "bias" in trial:
        bias = trial["bias"]
        box.set_bias_vector(saved_bias if bias is True else [0] * 6 if bias is False else bias)
    if "filter" in trial:
        box.set_filter(trial["filter"])
    if "buffer" in trial:
        box.set_buffer_records(trial["buffer"])
    return box.set_rate_hz(trial["rate"]) if "rate" in trial else box.communications()["rate_hz"]


# ── statistics ───────────────────────────────────────────────────────────────

def column(samples, index):
    return [s[index] for s in samples]


def mean(values):
    return sum(values) / len(values)


def sigma(values):
    average = mean(values)
    return (sum((v - average) ** 2 for v in values) / len(values)) ** 0.5


# ── the run ──────────────────────────────────────────────────────────────────

def run(trials=None, slot=None, host=None, seconds=None, slot_params=None,
        device_globals=None, restore_after=None, quiet=False):
    """Apply each trial, capture, and return a row per trial.

    Every argument defaults to the block at the top of this file, so `run()`
    with no arguments is what running the script does.
    """
    trials = TRIALS if trials is None else trials
    slot = SLOT if slot is None else slot
    host = HOST if host is None else host
    seconds = SECONDS if seconds is None else seconds
    slot_params = SLOT_PARAMS if slot_params is None else slot_params
    device_globals = GLOBALS if device_globals is None else device_globals
    restore_after = RESTORE if restore_after is None else restore_after

    if slot == WORKING_SLOT:
        raise ValueError("slot 0 holds the working configuration and is never written; "
                         "pick a scratch slot, e.g. 8")

    box = NetBoxConfig(host)
    say = (lambda *a: None) if quiet else print
    info = box.identity()
    say(f"Net F/T at {info['ip']}  firmware {info['firmware']}  "
        f"internal rate {info['internal_rate_hz']} Hz")

    saved = snapshot(box, slot)
    saved_bias = saved["settings"]["bias_vector"] or [0] * 6
    say(f"sandbox slot {slot} ({saved['slot']['name'] or 'unnamed'}), "
        f"active slot {saved['settings']['active_slot']}, bias {saved_bias}")
    say(f"{'restoring' if restore_after else 'KEEPING'} device state on exit\n")

    results = []
    try:
        apply_slot(box, slot, dict(slot_params))
        apply_globals(box, dict(device_globals))
        box.select_configuration(slot)
        config = box.configuration(slot)
        scale_f, scale_t = config["counts_per_force"], config["counts_per_torque"]
        say(f"slot {slot}: units {config['force_unit']}/{config['torque_unit']}, "
            f"tool transform {config['tool_transform']} ({config['tool_distance_unit']})\n")

        for trial in trials:
            effective = apply_trial(box, trial, saved_bias)
            time.sleep(0.2)                      # let the new settings settle
            samples, stats = capture(host, seconds, scale_f, scale_t)
            label = describe(trial)
            if not samples:
                say(f"  {label:<34} no data received")
                continue
            means = [mean(column(samples, i)) for i in range(6)]
            noises = [sigma(column(samples, i)) for i in range(6)]
            say(f"  {label:<34} {len(samples):>6} samples  "
                f"{stats['measured_hz']:>7.0f} Hz  {stats['dropped']:>4} dropped  "
                f"Fz mean={means[2]:+.4f} sigma={noises[2]:.4f} {config['force_unit']}")
            results.append({"label": label, "trial": dict(trial), "samples": samples,
                            "means": means, "sigma": noises, "rate": effective,
                            "dropped": stats["dropped"], "slot": slot,
                            "force_unit": config["force_unit"],
                            "torque_unit": config["torque_unit"]})
    finally:
        if restore_after:
            restore(box, slot, saved)
            back = box.settings()
            say(f"\nrestored: rate {box.communications()['rate_hz']} Hz, "
                f"filter code {back['filter_code']}, bias {back['bias_vector']}, "
                f"active slot {back['active_slot']}")
        else:
            say("\nRESTORE is off: device left as the last trial set it")
    return results


def report(results):
    if not results:
        return
    force, torque = results[0]["force_unit"], results[0]["torque_unit"]
    for title, key in (("mean (what bias moves)", "means"),
                       ("sigma (what the filter moves)", "sigma")):
        print(f"\n{title}   force in {force}, torque in {torque}")
        print(f"  {'trial':<34}" + "".join(f"{a:>10}" for a in AXES))
        for row in results:
            print(f"  {row['label']:<34}" + "".join(f"{v:>10.4f}" for v in row[key]))


def plot(results, save=None, show=None):
    if not results:
        return
    save = SAVE if save is None else save
    show = SHOW if show is None else show

    import numpy as np
    import matplotlib
    if save and not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    unit = results[0]["force_unit"]
    fig, axes = plt.subplots(len(results), 2, figsize=(13, 2.6 * len(results)), squeeze=False)
    for row, result in enumerate(results):
        # Mean is deliberately left in: the bias trials are only legible if the
        # offset is visible, which is the opposite of what the noise tools do.
        fz = np.array(column(result["samples"], 2))
        t = np.arange(len(fz)) / result["rate"]

        trace = axes[row][0]
        trace.plot(t, fz, linewidth=0.6)
        trace.axhline(result["means"][2], color="tab:red", linewidth=0.8, linestyle="--")
        trace.set_title(f"{result['label']}   Fz   mean={result['means'][2]:+.4f} "
                        f"sigma={result['sigma'][2]:.4f} {unit}", fontsize=9)
        trace.set_xlabel("time (s)", fontsize=8)
        trace.set_ylabel(f"Fz ({unit})", fontsize=8)
        trace.grid(alpha=0.3)

        spectrum = axes[row][1]
        centred = fz - fz.mean()
        if len(centred) > 16:
            window = np.hanning(len(centred))
            magnitude = np.abs(np.fft.rfft(centred * window)) / len(centred)
            freqs = np.fft.rfftfreq(len(centred), 1.0 / result["rate"])
            spectrum.semilogy(freqs[1:], magnitude[1:] + 1e-12, linewidth=0.6)
        spectrum.set_title(f"spectrum to Nyquist = {result['rate'] / 2:.0f} Hz", fontsize=9)
        spectrum.set_xlabel("frequency (Hz)", fontsize=8)
        spectrum.set_ylabel("magnitude", fontsize=8)
        spectrum.grid(alpha=0.3, which="both")

    fig.suptitle(f"Net F/T sandbox: slot {results[0]['slot']}", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.97))

    summary, bars = plt.subplots(1, 2, figsize=(13, 3.4))
    positions = np.arange(len(AXES))
    width = 0.8 / len(results)
    for index, result in enumerate(results):
        offset = (index - (len(results) - 1) / 2) * width
        bars[0].bar(positions + offset, result["means"], width, label=result["label"])
        bars[1].bar(positions + offset, result["sigma"], width, label=result["label"])
    for axis, title in zip(bars, ("mean per axis", "sigma per axis")):
        axis.set_xticks(positions)
        axis.set_xticklabels(AXES)
        axis.set_title(title, fontsize=10)
        axis.grid(alpha=0.3, axis="y")
    bars[1].set_yscale("log")
    bars[0].legend(fontsize=7)
    summary.tight_layout()

    if save:
        stem = Path(save)
        summary_path = stem.with_name(f"{stem.stem}_summary{stem.suffix}")
        fig.savefig(save, dpi=130)
        summary.savefig(summary_path, dpi=130)
        print(f"\nwrote {save} and {summary_path}")
    if show:
        plt.show()


if __name__ == "__main__":
    results = run()
    report(results)
    plot(results)
