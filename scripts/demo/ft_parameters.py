#!/usr/bin/env python3
"""Show what the Net F/T's RDT rate and low-pass filter actually do to the data.

Captures a short burst at several (output rate, filter) combinations, reports
noise per axis, and plots the traces and their spectra side by side.

The box always samples internally at 7 kHz and decimates to the RDT output rate
*without* filtering first, so a low output rate with no filter aliases
everything up to 3.5 kHz down into the band you can see. Raising the rate or
enabling a matched filter both address that, differently.

Every setting touched is captured at startup and restored on exit, including
after a failure or Ctrl-C. Configuration slots are never written.

Usage:
    python scripts/demo/ft_parameters.py --host 192.168.1.11 --seconds 2
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "bridge"))

from _rdt import AXES, capture  # noqa: E402
from robot_api.hardware.ft_config import FILTER_CUTOFF_HZ, NetBoxConfig  # noqa: E402


def noise(values):
    mean = sum(values) / len(values)
    return (sum((v - mean) ** 2 for v in values) / len(values)) ** 0.5


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="192.168.1.11")
    parser.add_argument("--seconds", type=float, default=2.0)
    parser.add_argument("--output", default="ft_parameters.png")
    parser.add_argument("--no-plot", action="store_true")
    args = parser.parse_args()

    box = NetBoxConfig(args.host)
    info = box.identity()
    config = box.configuration()
    print(f"Net F/T at {info['ip']}  firmware {info['firmware']}  "
          f"internal rate {info['internal_rate_hz']} Hz")
    print(f"active slot {config['active_slot']}  units {config['force_unit']}/{config['torque_unit']}  "
          f"counts per unit {config['counts_per_force']}/{config['counts_per_torque']}")
    print(f"tool transform {config['tool_transform']} "
          f"({config['tool_distance_unit']}, {config['tool_angle_unit']})")
    print(f"sensing ranges {config['sensing_ranges']}\n")

    original = box.communications()
    original_filter = box.settings()["filter_code"]
    scale_f = config["counts_per_force"]
    scale_t = config["counts_per_torque"]

    # (output rate, filter code): full rate unfiltered, full rate filtered,
    # then the low rate that aliases, with and without a matched filter.
    trials = [(7000, 0), (7000, 5), (100, 0), (100, 6)]
    results = []
    try:
        for rate_hz, filter_code in trials:
            effective = box.set_rate_hz(rate_hz)
            box.set_filter(filter_code)
            time.sleep(0.2)                      # let the new settings settle
            samples, stats = capture(args.host, args.seconds, scale_f, scale_t)
            dropped = stats["dropped"]
            if not samples:
                print(f"  {effective:>5} Hz filter {filter_code}: no data received")
                continue
            measured = len(samples) / args.seconds
            cutoff = FILTER_CUTOFF_HZ[filter_code]
            label = f"{effective} Hz, {'no filter' if cutoff is None else str(cutoff) + ' Hz LPF'}"
            noises = [noise([s[i] for s in samples]) for i in range(6)]
            print(f"  {label:<22} {len(samples):>6} samples  {measured:>7.0f} Hz measured  "
                  f"{dropped:>5} dropped  noise Fz={noises[2]:.4f} {config['force_unit']}")
            results.append((label, samples, noises, effective))
    finally:
        box.set_rate_hz(original["rate_hz"])
        box.set_buffer_records(original["buffer_records"])
        box.set_filter(original_filter)
        print(f"\nrestored: rate {box.communications()['rate_hz']} Hz, "
              f"filter code {box.settings()['filter_code']}")

    if results:
        print("\nnoise (standard deviation) per axis")
        print(f"  {'setting':<22}" + "".join(f"{a:>10}" for a in AXES))
        for label, _samples, noises, _rate in results:
            print(f"  {label:<22}" + "".join(f"{n:>10.4f}" for n in noises))

    if results and not args.no_plot:
        plot(results, config, args.output)


def plot(results, config, output):
    import numpy as np
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(len(results), 2, figsize=(13, 2.6 * len(results)), squeeze=False)
    unit = config["force_unit"]
    for row, (label, samples, noises, rate) in enumerate(results):
        fz = np.array([s[2] for s in samples])
        fz = fz - fz.mean()
        t = np.arange(len(fz)) / rate

        trace = axes[row][0]
        trace.plot(t, fz, linewidth=0.6)
        trace.set_title(f"{label}   Fz, mean removed   sigma={noises[2]:.4f} {unit}", fontsize=9)
        trace.set_xlabel("time (s)", fontsize=8)
        trace.set_ylabel(f"Fz ({unit})", fontsize=8)
        trace.grid(alpha=0.3)

        # Spectrum: what the filter and the output rate actually remove.
        spectrum = axes[row][1]
        if len(fz) > 16:
            window = np.hanning(len(fz))
            magnitude = np.abs(np.fft.rfft(fz * window)) / len(fz)
            freqs = np.fft.rfftfreq(len(fz), 1.0 / rate)
            spectrum.semilogy(freqs[1:], magnitude[1:] + 1e-12, linewidth=0.6)
        spectrum.set_title(f"spectrum to Nyquist = {rate / 2:.0f} Hz", fontsize=9)
        spectrum.set_xlabel("frequency (Hz)", fontsize=8)
        spectrum.set_ylabel("magnitude", fontsize=8)
        spectrum.grid(alpha=0.3, which="both")

    fig.suptitle("ATI Net F/T: effect of RDT output rate and low-pass filter", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(output, dpi=130)
    print(f"\nwrote {output}")


if __name__ == "__main__":
    main()
