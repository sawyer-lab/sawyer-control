"""
Title       : Experimental Trajectory Utilities
Module      : sawyer_control.experiments.trajectory
Description : Load and time-scale Sawyer joint trajectories while keeping a
              fixed robot command frequency.

Author      : Oscar Federico García Castro
Email       : oscar.garciac@cinvestav.mx
Affiliation : CINVESTAV, Robotics
Advisor     : Dr. Gustavo Arechavaleta

Created     : 2026-10-03
Updated     : 2026-10-03
Version     : 0.1.0
Python      : 3.8+
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class ResampledTrajectory:
    positions: tuple[tuple[float, ...], ...]
    metadata: dict[str, float | int | str]


def _read_csv(path: Path) -> tuple[np.ndarray, np.ndarray]:
    times: list[float] = []
    positions: list[tuple[float, ...]] = []
    with path.expanduser().open(newline='') as handle:
        for row in csv.reader(handle):
            if len(row) < 8:
                continue
            try:
                values = [float(value) for value in row[:8]]
            except ValueError:
                continue
            times.append(values[0])
            positions.append(tuple(values[1:8]))

    if len(positions) < 2:
        raise ValueError(f'{path}: need at least two rows of t,q1..q7')

    t = np.asarray(times, dtype=float)
    q = np.asarray(positions, dtype=float)
    if not np.all(np.isfinite(t)) or not np.all(np.isfinite(q)):
        raise ValueError(f'{path}: trajectory contains non-finite values')
    if np.any(np.diff(t) <= 0):
        raise ValueError(f'{path}: time column must be strictly increasing')
    return t, q


def resample_trajectory(
    path: Path | str,
    *,
    command_rate_hz: float,
    speed_scale: float = 1.0,
) -> ResampledTrajectory:
    """Resample q_s(t)=q(speed_scale*t) at a fixed command frequency."""
    source = Path(path).expanduser()
    if command_rate_hz <= 0 or not math.isfinite(command_rate_hz):
        raise ValueError('command_rate_hz must be finite and positive')
    if speed_scale <= 0 or not math.isfinite(speed_scale):
        raise ValueError('speed_scale must be finite and positive')

    times, positions = _read_csv(source)
    times = times - times[0]
    source_duration = float(times[-1])
    target_duration = source_duration / speed_scale

    intervals = max(1, int(math.ceil(target_duration * command_rate_hz)))
    played_times = np.arange(intervals + 1, dtype=float) / command_rate_hz
    source_times = np.minimum(played_times * speed_scale, source_duration)
    played = np.column_stack(
        [np.interp(source_times, times, positions[:, joint]) for joint in range(7)]
    )

    source_dt = np.diff(times)
    source_rate_hz = float(1.0 / np.median(source_dt))
    actual_duration = intervals / command_rate_hz
    rows = tuple(tuple(float(value) for value in row) for row in played)
    metadata: dict[str, float | int | str] = {
        'path': str(source),
        'source_rows': int(positions.shape[0]),
        'source_rate_hz': source_rate_hz,
        'source_duration_s': source_duration,
        'speed_scale': speed_scale,
        'requested_duration_s': target_duration,
        'played_rows': len(rows),
        'played_rate_hz': command_rate_hz,
        'played_duration_s': actual_duration,
        'time_scaling': 'q_scaled(t) = q_source(speed_scale * t)',
    }
    return ResampledTrajectory(rows, metadata)
