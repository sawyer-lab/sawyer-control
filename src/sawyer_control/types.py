from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Iterable


class ControlMode(str, Enum):
    POSITION = "position"
    VELOCITY = "velocity"
    TORQUE = "torque"
    TRAJECTORY = "trajectory"


@dataclass(frozen=True)
class JointVector:
    values: tuple[float, ...]

    def __init__(self, values: Iterable[float]):
        converted = tuple(float(value) for value in values)
        if len(converted) != 7:
            raise ValueError("Sawyer commands require exactly seven joint values")
        if not all(value == value and abs(value) != float("inf") for value in converted):
            raise ValueError("Joint values must be finite")
        object.__setattr__(self, "values", converted)


@dataclass(frozen=True)
class JointCommandSample:
    position: JointVector | None = None
    velocity: JointVector | None = None
    effort: JointVector | None = None
    acceleration: JointVector | None = None
