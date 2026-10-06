"""
Title       : Smooth Sawyer Motion
Module      : sawyer_control.experiments.motion
Description : Minimum-jerk joint-space transitions used before experiments.

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

import math
import time
from dataclasses import dataclass
from typing import Sequence

from sawyer_control import ControlMode, JointCommandSample, JointVector, SawyerRobotClient


@dataclass(frozen=True)
class SmoothMoveSettings:
    rate_hz: float = 100.0
    min_duration_s: float = 5.0
    max_speed_rad_s: float = 0.35
    tolerance_rad: float = 0.03
    hold_s: float = 0.5


def minimum_jerk(u: float) -> float:
    """Quintic blend with zero endpoint velocity and acceleration."""
    return 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5


def _validate_target(target: Sequence[float]) -> tuple[float, ...]:
    values = tuple(float(value) for value in target)
    if len(values) != 7:
        raise ValueError(f'Expected seven Sawyer joint positions, got {len(values)}')
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Target joint positions must be finite')
    return values


def smooth_joint_move(
    robot: SawyerRobotClient,
    target: Sequence[float],
    *,
    settings: SmoothMoveSettings = SmoothMoveSettings(),
) -> dict[str, float | int]:
    """Move to a target with a minimum-jerk interpolation in joint space."""
    target_values = _validate_target(target)
    if settings.rate_hz <= 0 or settings.min_duration_s <= 0:
        raise ValueError('Motion rate and minimum duration must be positive')
    if settings.max_speed_rad_s <= 0:
        raise ValueError('Maximum nominal joint speed must be positive')

    state = robot.get_state()
    start = tuple(float(value) for value in state.positions.values)
    if len(start) != 7:
        raise RuntimeError('Robot state does not contain seven joint positions')

    delta = tuple(desired - current for current, desired in zip(start, target_values))
    max_delta = max(abs(value) for value in delta)

    # max(ds/du) for s(u)=10u^3-15u^4+6u^5 is 1.875.
    speed_duration = 1.875 * max_delta / settings.max_speed_rad_s
    duration = max(settings.min_duration_s, speed_duration)
    steps = max(2, math.ceil(duration * settings.rate_hz))
    period_ns = round(1_000_000_000 / settings.rate_hz)
    started_ns = time.monotonic_ns()

    for index in range(1, steps + 1):
        deadline_ns = started_ns + index * period_ns
        delay_s = (deadline_ns - time.monotonic_ns()) / 1e9
        if delay_s > 0:
            time.sleep(delay_s)
        u = index / steps
        blend = minimum_jerk(u)
        position = JointVector(
            current + blend * change for current, change in zip(start, delta)
        )
        robot.command(JointCommandSample(position=position), ControlMode.POSITION)

    hold_deadline = time.monotonic() + settings.hold_s
    while time.monotonic() < hold_deadline:
        robot.command(
            JointCommandSample(position=JointVector(target_values)),
            ControlMode.POSITION,
        )
        time.sleep(1.0 / settings.rate_hz)

    final = tuple(float(value) for value in robot.get_state().positions.values)
    if len(final) != 7:
        raise RuntimeError('Robot state does not contain seven joint positions')
    error = max(abs(measured - desired) for measured, desired in zip(final, target_values))
    if error > settings.tolerance_rad:
        raise RuntimeError(
            f'Smooth move ended {error:.4f} rad from target; '
            f'tolerance is {settings.tolerance_rad:.4f} rad'
        )

    return {
        'duration_s': duration,
        'samples': steps,
        'final_error_rad': error,
    }
