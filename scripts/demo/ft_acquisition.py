#!/usr/bin/env python3
"""Diagnose acquisition timing through the bridge. Never commands robot motion."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src'))

from sawyer_control.experiments import StaticAcquisitionSettings, acquire_static


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bridge', default='127.0.0.1:50051')
    parser.add_argument('--duration', type=float, default=10.0)
    parser.add_argument('--poll-rate', type=float, default=100.0)
    parser.add_argument(
        '--window-ms',
        type=float,
        default=0.0,
        help='0: cursor intervals; >0: last completed bridge time window',
    )
    parser.add_argument('--max-age-ms', type=float, default=100.0)
    parser.add_argument('--timeout', type=float, default=0.25)
    parser.add_argument('--output', type=Path, default=Path('ft_acquisition.jsonl'))
    parser.add_argument('--sensor-rate', type=int, help='explicitly change device RDT rate')
    parser.add_argument('--filter-code', type=int, help='explicitly change device LPF code')
    parser.add_argument('--zero', action='store_true', help='explicitly bias at the current load')
    parser.add_argument(
        '--no-restore',
        action='store_true',
        help='leave explicitly changed rate/filter/bias in effect after acquisition',
    )
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    settings = StaticAcquisitionSettings(
        bridge=args.bridge,
        duration_s=args.duration,
        poll_rate_hz=args.poll_rate,
        window_ms=args.window_ms,
        max_age_ms=args.max_age_ms,
        timeout_s=args.timeout,
    )
    try:
        report = acquire_static(
            args.output,
            settings=settings,
            sensor_rate_hz=args.sensor_rate,
            filter_code=args.filter_code,
            zero=args.zero,
            restore_after=not args.no_restore,
        )
    except (OSError, ValueError, RuntimeError) as exc:
        print(f'Error: {exc}', file=sys.stderr)
        return 1
    print(json.dumps(report, indent=2))
    print(f'Wrote {args.output}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
