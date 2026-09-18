#!/usr/bin/env python3
"""Dynamic response test: what does a configuration do to a live signal?

The static survey answers "which setting is quietest" by measuring an untouched
sensor. That is only half the question - a hard enough filter makes any signal
beautifully clean by erasing whatever you cared about. This tool supplies the
other half, using a signal you generate by hand.

There is no protocol to follow and no axis to aim at. Start it, then tap, push,
twist, hold and release the tool however you can for the duration. It does not
look for individual events: it characterises the whole record, so irregular,
one-handed, multi-axis, whatever-you-can excitation is exactly as good as a
choreographed one. The only thing that helps is covering a range of speeds -
some sharp knocks, some slow pushes.

The comparison is against a gold-standard reference rather than against another
run: the capture is taken at the box's full 7 kHz, and the ideal band-limited
version of that signal at --target-hz is computed offline with a long zero-phase
filter. Every pipeline is then scored on how far it lands from that ideal:

  transfer    fraction of amplitude preserved, per frequency band - the direct
              answer to "does this setting still see what I did"
  rms error   distance from the ideal, including anything aliased in
  lag         measured by cross-correlation, not assumed
  floor       quietest half-second in the record, for the noise side of the trade

    python scripts/demo/ft_step.py --seconds 60
    python scripts/demo/ft_step.py --rate 1000 --filter 3 --save run_1k_152.npz
    python scripts/demo/ft_step.py --compare run_7k_raw.npz run_1k_152.npz
    python scripts/demo/ft_step.py --replot --save run_7k_raw.npz

Device settings are captured at startup and restored on exit, including after a
failure or Ctrl-C. Configuration slots are never written.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "bridge"))

from _rdt import AXES, stream  # noqa: E402
from ft_survey import _kernel_taps, _lowpass_kernel, pipe_filter_mean, pipe_mean  # noqa: E402
from robot_api.hardware.ft_config import FILTER_CUTOFF_HZ, NetBoxConfig  # noqa: E402

PIPELINES = ("subsample", "mean", "filter+mean")


# ── capture ──────────────────────────────────────────────────────────────────

def run_capture(host, seconds, scale_f, scale_t, lead_in=5.0):
    """One continuous stream, with a running clock so you can pace yourself."""
    print(f"\n  Tap, push, twist, press and release the tool for {seconds:g}s.")
    print("  Any direction, any rhythm. Mix sharp knocks with slow pushes -")
    print("  covering a range of speeds is the only thing that matters.\n")
    for remaining in range(int(lead_in), 0, -1):
        print(f"\r  starting in {remaining}... ", end="", flush=True)
        time.sleep(1)
    print("\r  GO" + " " * 30)

    rows, last_shown = [], None
    generator = stream(host, seconds, scale_f, scale_t)
    while True:
        try:
            moment, row = next(generator)
        except StopIteration as done:
            stats = done.value
            break
        left = int(seconds - moment) + 1
        if left != last_shown:
            last_shown = left
            print(f"\r  {left:3d}s left ", end="", flush=True)
        rows.append(row)
    print("\n  done.")
    return np.array(rows, dtype=float), stats


# ── pipelines with an honest time base ───────────────────────────────────────

def apply_pipeline(name, signal, ratio, rate_hz):
    """Return (times, values) at the target rate.

    The timestamps matter as much as the values: a block mean reports the centre
    of its window, not its trailing edge, and `filter+mean` trims half a kernel
    off the front. Getting those offsets wrong would fabricate or hide exactly
    the delay this test measures.
    """
    if ratio <= 1:
        return np.arange(len(signal)) / rate_hz, signal
    if name == "subsample":
        output = signal[::ratio]
        return np.arange(len(output)) * ratio / rate_hz, output
    if name == "mean":
        output = pipe_mean(signal, ratio)
        return (np.arange(len(output)) * ratio + (ratio - 1) / 2) / rate_hz, output
    output = pipe_filter_mean(signal, ratio)
    edge = _kernel_taps(ratio) // 2 if len(signal) > 3 * _kernel_taps(ratio) else 0
    return (edge + np.arange(len(output)) * ratio + (ratio - 1) / 2) / rate_hz, output


def extra_latency_ms(name, ratio, rate_hz):
    """Lookahead a live consumer pays for. `filter+mean` is zero phase, not zero
    delay: the symmetric kernel needs future samples."""
    if name == "filter+mean" and ratio > 1:
        return 1000.0 * (_kernel_taps(ratio) // 2) / rate_hz
    return 0.0


def ideal_reference(signal, rate_hz, target_hz, times):
    """The best possible target-rate view of what actually happened.

    A long zero-phase FIR just under the target Nyquist, evaluated at whatever
    instants a pipeline reports. Non-causal and far too expensive for a control
    loop, which is the point: it is the bound, not a candidate. Scoring against
    it separates two things a single noise number conflates - losing real signal
    to filtering, and folding out-of-band noise in by aliasing. Both show up as
    distance from this reference.
    """
    taps = 2049
    cutoff = 0.45 * target_hz / rate_hz
    offset = signal.mean()
    padded = np.concatenate([np.full(taps, signal[0] - offset), signal - offset,
                             np.full(taps, signal[-1] - offset)])
    filtered = np.convolve(padded, _lowpass_kernel(cutoff, taps), mode="same")
    filtered = filtered[taps:-taps] + offset
    return np.interp(times, np.arange(len(signal)) / rate_hz, filtered)


# ── metrics, none of which need an event ─────────────────────────────────────

def noise_floor(signal, rate_hz, window_s=0.5):
    """Quietest window in the record. Finds the moments you were not touching
    the sensor without being told when those were."""
    span = max(8, int(window_s * rate_hz))
    blocks = len(signal) // span
    if blocks < 2:
        return float(signal.std())
    trimmed = signal[:blocks * span].reshape(blocks, span)
    return float(trimmed.std(axis=1).min())


def transfer(reference, output, rate_hz, segment=256):
    """Amplitude preserved per frequency, by Welch-averaged cross-spectrum.

    Averaging over segments is what makes this work on hand-made excitation: any
    single segment is dominated by whatever you happened to be doing, but the
    ratio of cross-spectrum to reference power is an unbiased estimate of the
    pipeline's response regardless of what drove it.
    """
    count = min(len(reference), len(output))
    reference, output = reference[:count], output[:count]
    if count < 2 * segment:
        segment = max(32, count // 4)
    step = segment // 2
    window = np.hanning(segment)
    cross = np.zeros(segment // 2 + 1, dtype=complex)
    power = np.zeros(segment // 2 + 1)
    blocks = 0
    for start in range(0, count - segment + 1, step):
        a = np.fft.rfft((reference[start:start + segment] -
                         reference[start:start + segment].mean()) * window)
        b = np.fft.rfft((output[start:start + segment] -
                         output[start:start + segment].mean()) * window)
        cross += b * np.conj(a)
        power += np.abs(a) ** 2
        blocks += 1
    if not blocks:
        return np.array([]), np.array([]), np.array([])
    freqs = np.fft.rfftfreq(segment, 1.0 / rate_hz)
    strong = power > power.max() * 1e-4          # ignore bands you never excited
    magnitude = np.where(strong, np.abs(cross) / np.maximum(power, 1e-30), np.nan)
    return freqs, magnitude, power


def band_transfer(freqs, magnitude, low, high):
    band = (freqs >= low) & (freqs < high) & ~np.isnan(magnitude)
    return float(np.nanmean(magnitude[band])) if band.any() else float("nan")


def measured_lag_ms(reference, output, rate_hz, span=12):
    """Delay from cross-correlation rather than from theory, so a pipeline whose
    timestamps are wrong cannot hide it.

    The correlation peak is interpolated parabolically: at the target rate one
    whole sample is 10 ms, so without this the answer would only ever come back
    as a multiple of a tick and every sub-tick difference would read as zero.
    Interpolation is biased for small shifts - measured against known delays it
    lands within about a tenth of a tick, so read this as a coarse check that a
    pipeline's timing is what it claims, not as a precise latency figure.
    """
    count = min(len(reference), len(output))
    a = reference[:count] - reference[:count].mean()
    b = output[:count] - output[:count].mean()
    if not a.any() or not b.any():
        return float("nan")
    lags = np.arange(-span, span + 1)
    scores = np.array([np.dot(a[max(0, -lag):count - max(0, lag)],
                              b[max(0, lag):count - max(0, -lag)]) for lag in lags])
    peak = int(np.argmax(scores))
    shift = 0.0
    if 0 < peak < len(scores) - 1:
        left, middle, right = scores[peak - 1], scores[peak], scores[peak + 1]
        denominator = left - 2 * middle + right
        if denominator:
            shift = 0.5 * (left - right) / denominator
    return float(1000.0 * (lags[peak] + shift) / rate_hz)


# ── analysis ─────────────────────────────────────────────────────────────────

BANDS = ((0.0, 5.0), (5.0, 15.0), (15.0, 50.0))


def analyse(rows, rate_hz, target_hz, unit_f, unit_t):
    ratio = max(1, int(round(rate_hz / target_hz)))
    report, spectra = [], {}

    print(f"\noversampling ratio {rate_hz:g} Hz -> {target_hz:g} Hz = {ratio}x\n")
    header = (f"  {'axis':<4} {'floor':>9} {'activity':>9} {'moved':>7}")
    print(header + "   (activity = sigma over the whole record)")
    active = []
    for column, axis in enumerate(AXES):
        signal = rows[:, column]
        floor = noise_floor(signal, rate_hz)
        activity = float(signal.std())
        moved = activity / floor if floor else 0.0
        print(f"  {axis:<4} {floor:>9.5f} {activity:>9.5f} {moved:>6.0f}x")
        if moved > 3:
            active.append((column, axis, floor, activity))
    if not active:
        print("\n  nothing moved much above the noise floor - was the sensor touched?")
        return [], {}, ratio

    print(f"\nscoring {len(active)} excited axes against the ideal "
          f"{target_hz:g} Hz reference\n")
    for column, axis, floor, activity in active:
        signal = rows[:, column]
        unit = unit_f if column < 3 else unit_t
        for pipeline in PIPELINES:
            times, output = apply_pipeline(pipeline, signal, ratio, rate_hz)
            reference = ideal_reference(signal, rate_hz, target_hz, times)
            if len(output) < 32:
                continue
            error = output - reference
            freqs, magnitude, power = transfer(reference, output, target_hz)
            row = {
                "axis": axis, "pipeline": pipeline, "unit": unit,
                "floor": floor, "activity": activity,
                "rms_error": float(error.std()),
                "error_ratio": float(error.std() / activity) if activity else float("nan"),
                "peak_retention": (float(np.abs(output - np.median(output)).max() /
                                         np.abs(reference - np.median(reference)).max())
                                   if np.abs(reference - np.median(reference)).max() else
                                   float("nan")),
                "lag_ms": measured_lag_ms(reference, output, target_hz),
                "extra_latency_ms": extra_latency_ms(pipeline, ratio, rate_hz),
            }
            for low, high in BANDS:
                row[f"transfer_{low:g}_{high:g}"] = band_transfer(freqs, magnitude, low, high)
            covered = freqs[~np.isnan(magnitude)]
            row["excited_to_hz"] = float(covered.max()) if len(covered) else 0.0
            report.append(row)
            spectra[(axis, pipeline)] = (freqs, magnitude)
    return report, spectra, ratio


def print_report(report, target_hz):
    if not report:
        return
    print(f"  {'axis':<4} {'pipeline':<12} {'rms err':>9} {'err/sig':>8} "
          f"{'peak':>6} " + " ".join(f"{low:g}-{high:g}Hz".rjust(9) for low, high in BANDS) +
          f" {'lag':>8} {'+lag':>8}")
    print("  " + "-" * 96)
    for row in sorted(report, key=lambda r: (r["axis"], r["rms_error"])):
        bands = " ".join(f"{row[f'transfer_{low:g}_{high:g}']:>9.2f}" for low, high in BANDS)
        print(f"  {row['axis']:<4} {row['pipeline']:<12} {row['rms_error']:>9.5f} "
              f"{row['error_ratio']:>7.1%} {row['peak_retention']:>6.2f} {bands} "
              f"{row['lag_ms']:>5.1f} ms {row['extra_latency_ms']:>5.1f} ms")

    print("\n  averaged over excited axes:")
    for pipeline in PIPELINES:
        rows = [r for r in report if r["pipeline"] == pipeline]
        if not rows:
            continue
        low_band = np.nanmean([r[f"transfer_{BANDS[0][0]:g}_{BANDS[0][1]:g}"] for r in rows])
        high_band = np.nanmean([r[f"transfer_{BANDS[-1][0]:g}_{BANDS[-1][1]:g}"] for r in rows])
        print(f"    {pipeline:<12} error {np.mean([r['error_ratio'] for r in rows]):>6.1%} "
              f"of signal, keeps {low_band:.2f} of slow content and {high_band:.2f} "
              f"of {BANDS[-1][0]:g}-{BANDS[-1][1]:g} Hz, "
              f"lag {np.nanmean([r['lag_ms'] for r in rows]):+.1f} ms")

    covered = max(r["excited_to_hz"] for r in report)
    if covered < target_hz / 4:
        print(f"\n  NOTE: you only excited content up to {covered:.0f} Hz, so the "
              f"{BANDS[-1][0]:g}-{BANDS[-1][1]:g} Hz column is extrapolated from very "
              "little. Redo it with sharper knocks to judge fast content.")

    best = min(report, key=lambda r: r["error_ratio"])
    print(f"\n  closest to the ideal {target_hz:g} Hz signal: {best['pipeline']} "
          f"({best['error_ratio']:.1%} error on {best['axis']})")
    aliasing = [r for r in report if r["pipeline"] == "subsample"]
    if aliasing and np.mean([r["error_ratio"] for r in aliasing]) > 0.2:
        print(f"  subsample sits {np.mean([r['error_ratio'] for r in aliasing]):.0%} off the "
              "ideal - that gap is aliasing, and no amount of downstream smoothing "
              "removes it.")
    fast_loss = [r for r in report
                 if r[f"transfer_{BANDS[-1][0]:g}_{BANDS[-1][1]:g}"] < 0.5
                 and r["pipeline"] != "subsample"]
    if fast_loss:
        print(f"  NOTE: {len({r['pipeline'] for r in fast_loss})} pipeline(s) keep under half "
              f"the {BANDS[-1][0]:g}-{BANDS[-1][1]:g} Hz content. If fast contact "
              "transitions matter, carry a peak-hold per tick alongside the average.")


def plot(rows, report, spectra, rate_hz, target_hz, ratio, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 2, figsize=(15, 11))
    colours = {"subsample": "#e76f51", "mean": "#e9c46a", "filter+mean": "#2a9d8f"}
    busiest = max({r["axis"] for r in report},
                  key=lambda a: next(r["activity"] for r in report if r["axis"] == a))
    column = AXES.index(busiest)
    signal = rows[:, column]

    # the busiest half second, so the overlay shows the hardest case
    span = int(0.5 * rate_hz)
    blocks = len(signal) // span
    start = int(np.argmax(signal[:blocks * span].reshape(blocks, span).std(axis=1))) * span
    window = slice(start, start + span)
    times = np.arange(len(signal)) / rate_hz
    axes[0][0].plot(times[window], signal[window], color="#264653", linewidth=0.7,
                    label=f"{rate_hz:g} Hz truth")
    for pipeline in PIPELINES:
        pipe_times, pipe_values = apply_pipeline(pipeline, signal, ratio, rate_hz)
        select = (pipe_times >= times[window][0]) & (pipe_times <= times[window][-1])
        axes[0][0].plot(pipe_times[select], pipe_values[select], marker="o", markersize=2.5,
                        linewidth=1.0, color=colours[pipeline], label=pipeline)
    axes[0][0].set_xlabel("time (s)", fontsize=8)
    axes[0][0].set_ylabel(busiest, fontsize=8)
    axes[0][0].set_title(f"busiest half second on {busiest}: what each pipeline reports",
                         fontsize=9)
    axes[0][0].legend(fontsize=7)
    axes[0][0].grid(alpha=0.3)

    for (axis, pipeline), (freqs, magnitude) in spectra.items():
        if axis != busiest or not len(freqs):
            continue
        axes[0][1].plot(freqs[1:], magnitude[1:], color=colours[pipeline],
                        linewidth=1.2, label=pipeline)
    axes[0][1].axhline(1.0, color="k", linestyle="--", linewidth=0.8)
    axes[0][1].set_ylim(0, 1.4)
    axes[0][1].set_xlabel("frequency (Hz)", fontsize=8)
    axes[0][1].set_ylabel("amplitude preserved", fontsize=8)
    axes[0][1].set_title(f"how much of each frequency survives ({busiest}); "
                         "1.0 = intact", fontsize=9)
    axes[0][1].legend(fontsize=7)
    axes[0][1].grid(alpha=0.3)

    labels = [f"{r['axis']} + {r['pipeline']}" for r in report]
    axes[1][0].barh(range(len(report)), [r["error_ratio"] for r in report],
                    color=[colours[r["pipeline"]] for r in report])
    axes[1][0].set_yticks(range(len(report)))
    axes[1][0].set_yticklabels(labels, fontsize=6)
    axes[1][0].invert_yaxis()
    axes[1][0].set_xlabel("rms error / signal size", fontsize=8)
    axes[1][0].set_title(f"distance from the ideal {target_hz:g} Hz signal (lower is better)",
                         fontsize=9)
    axes[1][0].grid(alpha=0.3, axis="x")

    for pipeline in PIPELINES:
        rows_for = [r for r in report if r["pipeline"] == pipeline]
        axes[1][1].scatter([r["lag_ms"] + r["extra_latency_ms"] for r in rows_for],
                           [r[f"transfer_{BANDS[-1][0]:g}_{BANDS[-1][1]:g}"] for r in rows_for],
                           color=colours[pipeline], s=30, label=pipeline)
    axes[1][1].set_xlabel("total lag (ms)", fontsize=8)
    axes[1][1].set_ylabel(f"{BANDS[-1][0]:g}-{BANDS[-1][1]:g} Hz preserved", fontsize=8)
    axes[1][1].set_title("the actual trade: latency against fast-signal fidelity", fontsize=9)
    axes[1][1].legend(fontsize=7)
    axes[1][1].grid(alpha=0.3)

    fig.suptitle(f"Net F/T dynamic response - captured at {rate_hz:g} Hz, "
                 f"evaluated at {target_hz:g} Hz", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(output, dpi=130)
    print(f"wrote {output}")


# ── what you actually did ────────────────────────────────────────────────────

def _episodes(scaled, rate_hz, threshold_k, gap_s, min_duration_s):
    """Contiguous runs above threshold, with brief dips bridged."""
    active = scaled > threshold_k
    if not active.any():
        return []
    edges = np.diff(active.astype(int))
    starts = list(np.nonzero(edges == 1)[0] + 1)
    stops = list(np.nonzero(edges == -1)[0] + 1)
    if active[0]:
        starts.insert(0, 0)
    if active[-1]:
        stops.append(len(active))
    gap = int(gap_s * rate_hz)
    merged = [[starts[0], stops[0]]]
    for begin, end in zip(starts[1:], stops[1:]):
        if begin - merged[-1][1] <= gap:
            merged[-1][1] = end
        else:
            merged.append([begin, end])
    return [(begin, end) for begin, end in merged
            if end - begin >= min_duration_s * rate_hz]


def moments(rows, rate_hz, floors, threshold_k=10.0, span_fraction=0.02,
            gap_s=0.25, min_duration_s=0.05):
    """The distinct things you did to the sensor, for review only.

    Nothing in the scoring uses this - the metrics are computed over the whole
    record precisely so they do not depend on finding events. This exists so the
    capture can be eyeballed: which axis, when, how hard, and whether the run
    covered enough different kinds of input to trust the numbers.

    Each axis is segmented on its own. Taking only the single loudest axis at
    each instant would collapse a session into one long blob whenever something
    is always in contact, and hide the structure of which axis you were working
    when. Axes overlap here, which is the honest picture: pushing one direction
    loads the others too.

    The threshold is a multiple of the noise floor OR a small fraction of the
    record's own range, whichever is larger. A floor-only threshold is too
    sensitive to be useful on this hardware: the zero shifts by tens of noise
    floors after heavy loading, so an untouched sensor sitting at its new zero
    would read as one long event.
    """
    found = []
    for column, axis in enumerate(AXES):
        sigma = floors.get(axis) or 0.0
        if sigma <= 0:
            continue
        centred = rows[:, column] - np.median(rows[:, column])
        scaled = np.abs(centred) / sigma
        threshold = max(threshold_k, span_fraction * np.abs(centred).max() / sigma)
        for begin, end in _episodes(scaled, rate_hz, threshold, gap_s, min_duration_s):
            peak = begin + int(np.argmax(scaled[begin:end]))
            # fastest slope inside the episode, in units per second: separates a
            # knock from a lean without needing to classify either
            slope = float(np.abs(np.diff(centred[begin:end])).max() * rate_hz) if end - begin > 1 else 0.0
            found.append({"time_s": begin / rate_hz, "duration_s": (end - begin) / rate_hz,
                          "axis": axis, "strength": float(scaled[peak]),
                          "value": float(centred[peak]), "peak_s": peak / rate_hz,
                          "slope": slope})
    return sorted(found, key=lambda event: event["time_s"])


def plot_timeline(rows, rate_hz, floors, found, unit_f, unit_t, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    times = np.arange(len(rows)) / rate_hz
    step = max(1, int(rate_hz // 200))          # thin for drawing only
    fig, axes = plt.subplots(6, 1, figsize=(15, 12), sharex=True)
    for column, (axis, panel) in enumerate(zip(AXES, axes)):
        signal = rows[:, column]
        panel.plot(times[::step], signal[::step], linewidth=0.5, color="#264653")
        # episodes as a bar along the top rather than shading the panel: with
        # something in contact most of the time, full-height shading covers the
        # trace and shows nothing
        low, high = signal.min(), signal.max()
        for event in found:
            if event["axis"] != axis:
                continue
            panel.plot([event["time_s"], event["time_s"] + event["duration_s"]],
                       [high + 0.06 * (high - low)] * 2, color="#e76f51", linewidth=3,
                       solid_capstyle="butt", clip_on=False)
        panel.set_ylabel(f"{axis} ({unit_f if column < 3 else unit_t})", fontsize=8)
        panel.grid(alpha=0.3)
        panel.tick_params(labelsize=7)
    axes[-1].set_xlabel("time (s)", fontsize=8)
    axes[0].set_title(f"the whole capture; red bars mark where each axis was loaded "
                      f"above 10x its own noise floor ({len(found)} episodes)", fontsize=10)
    fig.tight_layout()
    fig.savefig(output, dpi=120)
    print(f"wrote {output}")


def zero_shift(rows, rate_hz, floors, quiet_s=4.0, quiet_k=3.0):
    """Compare the unloaded reading at the start of the run with the end.

    Both windows are checked for being genuinely quiet first. Without that the
    comparison is worthless: a run that ends mid-push reports the push as drift.

    The change is split into magnitude and direction, because those have
    completely different causes. The sensor's own zero drifting changes |F|.
    The tool hanging at a different angle - the arm settling somewhere else
    after being leant on - rotates the gravity vector without changing its
    length, moving every component while |F| holds. Only the first is the
    sensor's fault, and only the first is unfixable by knowing the pose.
    """
    span = int(quiet_s * rate_hz)
    if len(rows) < 3 * span:
        return {}
    windows = {"start": rows[:span], "end": rows[-span:]}
    noisy = [name for name, window in windows.items()
             if any(window[:, column].std() > quiet_k * (floors.get(axis) or np.inf)
                    for column, axis in enumerate(AXES))]
    if noisy:
        return {"unusable": noisy}

    before = windows["start"].mean(axis=0)
    after = windows["end"].mean(axis=0)
    result = {"per_axis": {axis: {"shift": float(after[column] - before[column]),
                                 "floors": (float(abs(after[column] - before[column]) /
                                                  floors[axis]) if floors.get(axis) else 0.0)}
                          for column, axis in enumerate(AXES)}}
    for name, part in (("force", slice(0, 3)), ("torque", slice(3, 6))):
        first, last = before[part], after[part]
        length, new_length = float(np.linalg.norm(first)), float(np.linalg.norm(last))
        if length <= 0 or new_length <= 0:
            continue
        cosine = float(np.clip(first @ last / (length * new_length), -1.0, 1.0))
        result[name] = {"magnitude": length, "new_magnitude": new_length,
                        "relative": (new_length - length) / length,
                        "rotation_deg": float(np.degrees(np.arccos(cosine))),
                        "largest_component": float(np.abs(last - first).max())}
    return result


def print_moments(found, floors, unit_f, unit_t, shifts=None):
    print(f"\n{len(found)} episodes above 10x the noise floor, per axis "
          "(axes overlap - one push loads several):\n")
    print(f"  {'start':>7} {'lasted':>8} {'axis':<5} {'peak':>12} {'vs floor':>9} "
          f"{'fastest':>12}")
    print("  " + "-" * 60)
    for event in found:
        unit = unit_f if event["axis"] in AXES[:3] else unit_t
        print(f"  {event['time_s']:>6.2f}s {event['duration_s']:>7.2f}s {event['axis']:<5} "
              f"{event['value']:>+8.3f} {unit:<3} {event['strength']:>8.0f}x "
              f"{event['slope']:>9.0f} {unit}/s")
    by_axis = {}
    for event in found:
        by_axis.setdefault(event["axis"], []).append(event)
    print("\n  per axis: " + "   ".join(
        f"{axis} {len(events)} episodes, {sum(e['duration_s'] for e in events):.0f}s active"
        for axis, events in sorted(by_axis.items())))
    if not shifts:
        return
    if "unusable" in shifts:
        print(f"\n  zero check skipped: the {' and '.join(shifts['unusable'])} of the run "
              "was not quiet, so there is no unloaded reading to compare against. "
              "Leave a few still seconds at both ends to get this.")
        return

    print("\n  unloaded reading, start of run vs end:")
    for axis, shift in shifts["per_axis"].items():
        unit = unit_f if axis in AXES[:3] else unit_t
        print(f"    {axis:<3} {shift['shift']:>+8.4f} {unit:<3} "
              f"({shift['floors']:>4.0f}x the noise floor)")
    for name, unit in (("force", unit_f), ("torque", unit_t)):
        part = shifts.get(name)
        if not part:
            continue
        print(f"    |{name}| {part['magnitude']:.4f} -> {part['new_magnitude']:.4f} {unit} "
              f"({part['relative']:+.2%}), direction rotated "
              f"{part['rotation_deg']:.2f} deg, largest component moved "
              f"{part['largest_component']:.4f} {unit}")
    force = shifts.get("force")
    if force and abs(force["relative"]) < 0.01 and force["rotation_deg"] > 0.5:
        print("    Components moved but the magnitude held: that is the gravity vector "
              "rotating, not the sensor drifting. The tool ended up hanging at a "
              "different angle. Re-bias per pose, or compensate gravity from the "
              "known pose - averaging cannot help either way.")
    elif force and abs(force["relative"]) > 0.02:
        print("    The magnitude itself changed, which pose alone does not explain: "
              "either the sensor zero moved or something is still resting on the tool.")


# ── cross-run comparison ─────────────────────────────────────────────────────

def quiet_blocks(signal, rate_hz, window_s=0.5, keep=0.2):
    """The quietest `keep` fraction of the record, concatenated.

    Finds the moments you were not touching the sensor without being told when
    those were, so the noise spectrum can be measured from a hand-driven capture.
    """
    span = max(8, int(window_s * rate_hz))
    blocks = len(signal) // span
    if blocks < 4:
        return signal
    trimmed = signal[:blocks * span].reshape(blocks, span)
    order = np.argsort(trimmed.std(axis=1))
    return trimmed[order[:max(2, int(blocks * keep))]].ravel()


def compare(paths, target_hz):
    """Compare runs taken under different device settings.

    You cannot excite the sensor identically twice, so nothing measured from the
    part of the record where you were pushing can cross between runs - including
    any bandwidth measure taken over the whole capture, which mostly reports how
    fast you happened to move. Both columns here are therefore measured on the
    *quiet* stretches only: the noise floor, and how much of that noise sits in
    the upper part of the consumer's band. Noise is broadband whatever you did
    with your hands, so its spectrum is a property of the configuration.
    """
    print(f"\n{'run':<28} {'rate':>7} {'filter':>8} {'floor':>10} {'hf noise':>11}")
    print("-" * 68)
    for path in paths:
        archive = np.load(path, allow_pickle=True)
        rows, rate_hz = archive["rows"], float(archive["rate_hz"])
        cutoff = archive["filter_cutoff_hz"].item()
        floors = [noise_floor(rows[:, column], rate_hz) for column in range(6)]
        fractions = []
        for column in range(6):
            quiet = quiet_blocks(rows[:, column], rate_hz)
            spectrum = np.abs(np.fft.rfft((quiet - quiet.mean()) * np.hanning(len(quiet))))
            freqs = np.fft.rfftfreq(len(quiet), 1.0 / rate_hz)
            in_band = (freqs > 0) & (freqs <= target_hz / 2)
            fast = (freqs >= target_hz / 20) & (freqs <= target_hz / 2)
            if in_band.any() and spectrum[in_band].sum():
                fractions.append(float(spectrum[fast].sum() / spectrum[in_band].sum()))
        print(f"{Path(path).name:<28} {rate_hz:>7.0f} "
              f"{('raw' if cutoff is None else f'{cutoff} Hz'):>8} "
              f"{np.median(floors):>10.5f} {np.median(fractions):>11.3f}")
    print(f"\nfloor is the quietest stretch of each run; hf noise is the share of that "
          f"noise sitting between {target_hz / 20:g} and {target_hz / 2:g} Hz. A lower "
          "floor with an unchanged share means the configuration is simply quieter; the "
          "share dropping too means it is also rolling the top of your band off.")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="192.168.1.11")
    parser.add_argument("--seconds", type=float, default=60.0,
                        help="how long to keep working the sensor")
    parser.add_argument("--target-hz", type=float, default=100.0,
                        help="the rate your consumer actually runs at")
    parser.add_argument("--rate", type=int, default=7000, help="device output rate for this run")
    parser.add_argument("--filter", type=int, default=0, help="device filter code for this run")
    parser.add_argument("--lead-in", type=float, default=5.0)
    parser.add_argument("--output", default="ft_step.png")
    parser.add_argument("--save", default="ft_step_raw.npz")
    parser.add_argument("--no-plot", action="store_true")
    parser.add_argument("--replot", action="store_true",
                        help="re-analyse --save from a previous run; no device access")
    parser.add_argument("--timeline", action="store_true",
                        help="with --replot, also list and plot the strongest moments")
    parser.add_argument("--compare", nargs="+", metavar="NPZ",
                        help="compare saved runs instead of capturing; no device access")
    args = parser.parse_args()

    if args.compare:
        compare(args.compare, args.target_hz)
        return

    if args.replot:
        archive = np.load(args.save, allow_pickle=True)
        rows, rate_hz = archive["rows"], float(archive["rate_hz"])
        unit_f, unit_t = str(archive["force_unit"]), str(archive["torque_unit"])
        report, spectra, ratio = analyse(rows, rate_hz, args.target_hz, unit_f, unit_t)
        print_report(report, args.target_hz)
        if report and not args.no_plot:
            plot(rows, report, spectra, rate_hz, args.target_hz, ratio, args.output)
        if args.timeline:
            floors = {axis: noise_floor(rows[:, column], rate_hz)
                      for column, axis in enumerate(AXES)}
            found = moments(rows, rate_hz, floors)
            print_moments(found, floors, unit_f, unit_t,
                          zero_shift(rows, rate_hz, floors))
            if not args.no_plot:
                plot_timeline(rows, rate_hz, floors, found, unit_f, unit_t,
                              args.output.replace(".png", "_timeline.png"))
        return

    box = NetBoxConfig(args.host)
    identity = box.identity()
    config = box.configuration()
    scale_f, scale_t = config["counts_per_force"], config["counts_per_torque"]
    original = box.communications()
    original_filter = box.settings()["filter_code"]
    print(f"Net F/T {identity['ip']} fw {identity['firmware']}  "
          f"slot {config['active_slot']} units {config['force_unit']}/{config['torque_unit']}")

    try:
        effective = box.set_rate_hz(args.rate)
        box.set_filter(args.filter)
        time.sleep(0.3)
        cutoff = FILTER_CUTOFF_HZ[args.filter]
        print(f"capturing at {effective} Hz, filter "
              f"{'off' if cutoff is None else f'{cutoff} Hz'}")
        rows, stats = run_capture(args.host, args.seconds, scale_f, scale_t, args.lead_in)
    finally:
        box.set_rate_hz(original["rate_hz"])
        box.set_filter(original_filter)
        print(f"  restored: {box.communications()['rate_hz']} Hz, "
              f"filter code {box.settings()['filter_code']}")

    print(f"  {stats['samples']} samples, {stats['measured_hz']:.0f} Hz measured, "
          f"{stats['dropped']} dropped")
    if stats["samples"] < 1000:
        print("too little data to analyse")
        return

    if args.save:
        np.savez_compressed(args.save, rows=rows, rate_hz=effective,
                            filter_code=args.filter,
                            filter_cutoff_hz=np.array(cutoff, dtype=object),
                            force_unit=config["force_unit"], torque_unit=config["torque_unit"])
        print(f"  raw capture -> {args.save}")

    report, spectra, ratio = analyse(rows, effective, args.target_hz,
                                     config["force_unit"], config["torque_unit"])
    print_report(report, args.target_hz)
    if report and not args.no_plot:
        plot(rows, report, spectra, effective, args.target_hz, ratio, args.output)


if __name__ == "__main__":
    main()
