#!/usr/bin/env python3
"""Find the cleanest sensor configuration for a fixed consumer rate.

The question this answers is *not* "what output rate should the box run at" but
"which combination of device settings and software processing gives the quietest
signal at the rate my control loop actually consumes". Those have different
answers: running the box at 100 Hz throws away 69 of every 70 internal samples,
while running at 7 kHz and averaging them down recovers noise as sqrt(N) - but
only if the noise is white, which is what this tool measures rather than
assumes.

Every candidate is evaluated *after* decimation to --target-hz, so the numbers
are directly comparable. Reported per candidate:

  sigma       noise at the target rate (the headline number)
  efficiency  measured noise reduction vs the ideal sqrt(N); ~1.0 means white
              noise and averaging pays off, <<1 means structured noise
  delay       group delay the software pipeline adds, in ms
  Allan deviation, to show where averaging stops helping and drift takes over
  spectrum, to expose mains pickup, drift and the transducer resonance

Device settings are captured at startup and restored on exit, including after a
failure or Ctrl-C. Configuration slots are never written.

    python scripts/demo/ft_survey.py --seconds 10 --target-hz 100

Raw captures are saved to --save so analysis can be re-run without re-capturing
and so a later step-response test can reuse them.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "bridge"))

from _rdt import AXES, capture  # noqa: E402
from robot_api.hardware.ft_config import FILTER_CUTOFF_HZ, NetBoxConfig  # noqa: E402

# (output rate, filter code). Spans the oversampling ratios worth distinguishing
# at a 100 Hz consumer: 70x, 35x, 10x, 1x, with and without hardware filtering.
CANDIDATES = [
    (7000, 0), (7000, 1), (7000, 3),
    (3500, 0), (1000, 0), (1000, 3),
    (100, 0), (100, 6),
]


# ── software pipelines, all producing a target_hz signal ─────────────────────

def pipe_subsample(signal, ratio):
    """Take every Nth sample. What a consumer that polls a cache effectively
    gets: no averaging, so no noise benefit and full aliasing."""
    return signal[::ratio] if ratio > 1 else signal


def pipe_mean(signal, ratio):
    """Block average. The cheap, causal way to oversample; delays by half a
    block."""
    if ratio <= 1:
        return signal
    usable = len(signal) - (len(signal) % ratio)
    return signal[:usable].reshape(-1, ratio).mean(axis=1)


def _kernel_taps(ratio):
    """A narrower cutoff needs a longer kernel; scale with the decimation ratio
    rather than fixing a length that is too short at high ratios."""
    return int(max(31, min(1023, 8 * ratio))) | 1


def _lowpass_kernel(cutoff_normalised, taps):
    """Symmetric windowed-sinc FIR. Symmetric means linear phase, and taking the
    centred output makes it zero-phase - no scipy required."""
    n = np.arange(taps) - (taps - 1) / 2
    kernel = np.sinc(2 * cutoff_normalised * n) * np.hamming(taps)
    return kernel / kernel.sum()


def pipe_filter_mean(signal, ratio):
    """Anti-alias with a zero-phase FIR just below the target Nyquist, then
    average. Textbook decimation.

    The mean is removed before convolving and the kernel-length edges are
    discarded afterwards: `np.convolve` zero-pads, so a signal with a large DC
    offset produces enormous edge transients that would otherwise dominate the
    measured noise.
    """
    if ratio <= 1:
        return signal
    taps = _kernel_taps(ratio)
    if len(signal) <= 3 * taps:
        return pipe_mean(signal, ratio)
    offset = signal.mean()
    filtered = np.convolve(signal - offset, _lowpass_kernel(0.4 / ratio, taps), mode="same")
    edge = taps // 2
    return pipe_mean(filtered[edge:-edge] + offset, ratio)


# Extra latency beyond the one target tick every pipeline already waits for.
# None of these distort phase: subsampling is instantaneous, the block mean is
# linear-phase over exactly one tick, and the symmetric FIR is zero-phase - but
# the FIR must see half a kernel of future samples, which a live consumer pays
# for in lag.
PIPELINES = {
    "subsample": (pipe_subsample, lambda ratio, hz: 0.0),
    "mean": (pipe_mean, lambda ratio, hz: 0.0),
    "filter+mean": (pipe_filter_mean,
                    lambda ratio, hz: 0.0 if ratio <= 1 else 1000.0 * (_kernel_taps(ratio) // 2) / hz),
}


# ── metrics ──────────────────────────────────────────────────────────────────

def allan_deviation(signal, rate_hz, points=24):
    """Overlapping Allan deviation. Falls as 1/sqrt(tau) while noise is white;
    flattens or rises where drift and bias instability take over, which is the
    point past which longer averaging stops helping."""
    taus, deviations = [], []
    max_block = max(2, len(signal) // 8)
    for block in np.unique(np.geomspace(1, max_block, points).astype(int)):
        usable = len(signal) - (len(signal) % block)
        if usable < 4 * block:
            continue
        means = signal[:usable].reshape(-1, block).mean(axis=1)
        if len(means) < 3:
            continue
        deviation = np.sqrt(0.5 * np.mean(np.diff(means) ** 2))
        taus.append(block / rate_hz)
        deviations.append(deviation)
    return np.array(taus), np.array(deviations)


def spectrum(signal, rate_hz):
    centred = signal - signal.mean()
    window = np.hanning(len(centred))
    magnitude = np.abs(np.fft.rfft(centred * window)) / len(centred)
    return np.fft.rfftfreq(len(centred), 1.0 / rate_hz), magnitude


def line_pickup(freqs, magnitude):
    """Ratio of the worst 50/60 Hz peak to the local background. Mains pickup is
    the usual reason averaging underperforms sqrt(N)."""
    worst = 0.0
    for mains in (50.0, 60.0):
        near = (freqs > mains - 2) & (freqs < mains + 2)
        background = (freqs > mains - 12) & (freqs < mains + 12) & ~near
        if near.any() and background.any() and magnitude[background].mean() > 0:
            worst = max(worst, magnitude[near].max() / magnitude[background].mean())
    return worst


# ── survey ───────────────────────────────────────────────────────────────────

def survey(box, host, seconds, target_hz, axis_index, save_path):
    original = box.communications()
    original_filter = box.settings()["filter_code"]
    config = box.configuration()
    scale_f = config["counts_per_force"]
    scale_t = config["counts_per_torque"]
    unit = config["force_unit"] if axis_index < 3 else config["torque_unit"]

    results, captures = [], {}
    try:
        for rate_hz, filter_code in CANDIDATES:
            effective = box.set_rate_hz(rate_hz)
            box.set_filter(filter_code)
            time.sleep(0.3)                          # let the settings settle
            rows, stats = capture(host, seconds, scale_f, scale_t)
            cutoff = FILTER_CUTOFF_HZ[filter_code]
            label = f"{effective:>4} Hz / {'raw' if cutoff is None else f'{cutoff} Hz'}"
            if stats["samples"] < 10:
                print(f"  {label}: no data")
                continue

            signal = np.array([row[axis_index] for row in rows], dtype=float)
            captures[label] = signal
            ratio = max(1, int(round(effective / target_hz)))
            raw_sigma = float(signal.std())

            print(f"  {label}  {stats['samples']:>6} samples  "
                  f"{stats['measured_hz']:>7.0f} Hz  {stats['dropped']:>4} dropped  "
                  f"raw sigma={raw_sigma:.5f} {unit}")

            for name, (pipeline, delay_of) in PIPELINES.items():
                output = pipeline(signal, ratio)
                if len(output) < 8:
                    continue
                sigma = float(np.std(output))
                ideal = raw_sigma / np.sqrt(ratio)
                freqs, magnitude = spectrum(signal, effective)
                results.append({
                    "label": label, "pipeline": name, "rate_hz": effective,
                    "filter_code": filter_code, "filter_cutoff_hz": cutoff,
                    "ratio": ratio, "raw_sigma": raw_sigma, "sigma": sigma,
                    "ideal_sigma": float(ideal),
                    "efficiency": float(ideal / sigma) if sigma else 0.0,
                    "extra_latency_ms": float(delay_of(ratio, effective)),
                    "dropped": stats["dropped"], "measured_hz": stats["measured_hz"],
                    "line_pickup": float(line_pickup(freqs, magnitude)),
                })
    finally:
        box.set_rate_hz(original["rate_hz"])
        box.set_buffer_records(original["buffer_records"])
        box.set_filter(original_filter)
        print(f"\nrestored: {box.communications()['rate_hz']} Hz, "
              f"filter code {box.settings()['filter_code']}")

    if save_path and captures:
        np.savez_compressed(save_path, **{k.replace(" ", "_"): v for k, v in captures.items()})
        print(f"raw captures -> {save_path}")
    return results, captures, unit


def report(results, target_hz, unit):
    print(f"\nEvaluated at {target_hz} Hz, best first "
          f"(efficiency 1.0 = noise averages down as sqrt(N), i.e. it is white)\n")
    header = (f"  {'config':<20} {'pipeline':<12} {'N':>4} {'sigma':>10} "
              f"{'vs raw':>8} {'eff':>6} {'+lag':>9}")
    print(header)
    print("  " + "-" * (len(header) - 2))
    for row in sorted(results, key=lambda r: r["sigma"]):
        improvement = row["raw_sigma"] / row["sigma"] if row["sigma"] else 0
        print(f"  {row['label']:<20} {row['pipeline']:<12} {row['ratio']:>4} "
              f"{row['sigma']:>10.5f} {improvement:>7.1f}x {row['efficiency']:>6.2f} "
              f"{row['extra_latency_ms']:>6.1f} ms")

    best = min(results, key=lambda r: r["sigma"])
    zero_delay = [r for r in results if r["extra_latency_ms"] < 0.01]
    best_live = min(zero_delay, key=lambda r: r["sigma"]) if zero_delay else best
    worst = max(results, key=lambda r: r["sigma"])

    print(f"\n  quietest overall : {best['label']} + {best['pipeline']} "
          f"-> {best['sigma']:.5f} {unit}")
    print(f"  quietest with no extra lag: {best_live['label']} + {best_live['pipeline']} "
          f"-> {best_live['sigma']:.5f} {unit}")
    print(f"  worst            : {worst['label']} + {worst['pipeline']} "
          f"-> {worst['sigma']:.5f} {unit} ({worst['sigma'] / best['sigma']:.1f}x noisier)")

    pickup = max(r["line_pickup"] for r in results)
    if pickup > 3:
        print(f"\n  NOTE: mains pickup detected ({pickup:.1f}x background at 50/60 Hz). "
              "Averaging will underperform sqrt(N) until that is dealt with.")
    poor = [r for r in results if r["pipeline"] == "mean" and r["ratio"] > 4
            and r["efficiency"] < 0.7]
    if poor:
        print("  NOTE: averaging underperforms the sqrt(N) ideal on "
              f"{len(poor)} candidate(s) - the noise is not white there.")


def plot(results, captures, target_hz, unit, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 2, figsize=(15, 11))

    ranked = sorted(results, key=lambda r: r["sigma"])[:16]
    labels = [f"{r['label']} + {r['pipeline']}" for r in ranked]
    sigmas = [r["sigma"] for r in ranked]
    colours = ["#2a9d8f" if r["extra_latency_ms"] < 0.01 else "#e9c46a" for r in ranked]
    axes[0][0].barh(range(len(ranked)), sigmas, color=colours)
    axes[0][0].set_yticks(range(len(ranked)))
    axes[0][0].set_yticklabels(labels, fontsize=7)
    axes[0][0].invert_yaxis()
    axes[0][0].set_xlabel(f"sigma at {target_hz} Hz ({unit})", fontsize=8)
    axes[0][0].set_title("noise at the consumer's rate, best 16 (green = no extra lag)", fontsize=9)
    axes[0][0].grid(alpha=0.3, axis="x")

    for label, signal in captures.items():
        rate = float(label.split()[0])
        taus, deviations = allan_deviation(signal, rate)
        if len(taus):
            axes[0][1].loglog(taus, deviations, linewidth=0.9, label=label)
    axes[0][1].axvline(1.0 / target_hz, color="k", linestyle=":", linewidth=0.8)
    axes[0][1].set_xlabel("averaging time tau (s)", fontsize=8)
    axes[0][1].set_ylabel(f"Allan deviation ({unit})", fontsize=8)
    axes[0][1].set_title(f"where averaging stops paying (dotted = {target_hz} Hz tick)", fontsize=9)
    axes[0][1].legend(fontsize=5)
    axes[0][1].grid(alpha=0.3, which="both")

    for label, signal in captures.items():
        rate = float(label.split()[0])
        freqs, magnitude = spectrum(signal, rate)
        if len(freqs) > 2:
            axes[1][0].loglog(freqs[1:], magnitude[1:], linewidth=0.6, label=label)
    axes[1][0].axvline(target_hz / 2, color="k", linestyle=":", linewidth=0.8)
    axes[1][0].set_xlabel("frequency (Hz)", fontsize=8)
    axes[1][0].set_ylabel("magnitude", fontsize=8)
    axes[1][0].set_title(f"noise spectra (dotted = {target_hz / 2:.0f} Hz target Nyquist)", fontsize=9)
    axes[1][0].legend(fontsize=5)
    axes[1][0].grid(alpha=0.3, which="both")

    means = [r for r in results if r["pipeline"] in ("mean", "filter+mean")]
    axes[1][1].scatter([r["ratio"] for r in means], [r["efficiency"] for r in means],
                       c=["#2a9d8f" if r["pipeline"] == "filter+mean" else "#e76f51"
                          for r in means], s=22)
    axes[1][1].axhline(1.0, color="k", linestyle="--", linewidth=0.8)
    axes[1][1].set_xscale("log")
    axes[1][1].set_xlabel("oversampling ratio N", fontsize=8)
    axes[1][1].set_ylabel("measured / ideal sqrt(N)", fontsize=8)
    axes[1][1].set_title("is the noise white? (1.0 = averaging pays in full)", fontsize=9)
    axes[1][1].grid(alpha=0.3)

    fig.suptitle(f"Net F/T configuration survey - evaluated at {target_hz} Hz", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(output, dpi=130)
    print(f"wrote {output}")


def replot(save_path, json_path, target_hz, unit, output):
    """Re-run the report and plots from a previous capture, without touching
    the device."""
    results = json.loads(Path(json_path).read_text())
    archive = np.load(save_path)
    captures = {key.replace("_", " "): archive[key] for key in archive.files}
    report(results, target_hz, unit)
    plot(results, captures, target_hz, unit, output)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="192.168.1.11")
    parser.add_argument("--seconds", type=float, default=10.0,
                        help="capture length per candidate")
    parser.add_argument("--target-hz", type=float, default=100.0,
                        help="the rate your consumer actually runs at")
    parser.add_argument("--axis", default="Fz", choices=AXES)
    parser.add_argument("--output", default="ft_survey.png")
    parser.add_argument("--save", default="ft_survey_raw.npz")
    parser.add_argument("--json", default="")
    parser.add_argument("--no-plot", action="store_true")
    parser.add_argument("--replot", action="store_true",
                        help="re-analyse --save/--json from a previous run; no device access")
    args = parser.parse_args()

    if args.replot:
        replot(args.save, args.json or "ft_survey.json", args.target_hz, "N", args.output)
        return

    box = NetBoxConfig(args.host)
    identity = box.identity()
    config = box.configuration()
    print(f"Net F/T {identity['ip']} fw {identity['firmware']}  "
          f"slot {config['active_slot']} units {config['force_unit']}/{config['torque_unit']}")
    print(f"surveying {len(CANDIDATES)} candidates x {args.seconds:g}s on {args.axis}, "
          f"evaluated at {args.target_hz:g} Hz\n")

    results, captures, unit = survey(box, args.host, args.seconds, args.target_hz,
                                     AXES.index(args.axis), args.save)
    if not results:
        print("no results")
        return
    report(results, args.target_hz, unit)
    if args.json:
        Path(args.json).write_text(json.dumps(results, indent=2))
        print(f"metrics -> {args.json}")
    if not args.no_plot:
        plot(results, captures, args.target_hz, unit, args.output)


if __name__ == "__main__":
    main()
