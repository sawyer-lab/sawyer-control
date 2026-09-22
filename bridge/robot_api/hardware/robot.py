"""Small bridge-facing facade over the Sawyer ROS adapter."""

from __future__ import annotations

from .sawyer import SawyerInterface


class Robot:
    def __init__(self):
        self._sawyer = SawyerInterface()

    def move_to_joints(self, angles, timeout: float) -> bool:
        return self._sawyer.move_to_joint_positions(angles, timeout)

    def get_state(self) -> dict:
        return self._sawyer.state_snapshot()
