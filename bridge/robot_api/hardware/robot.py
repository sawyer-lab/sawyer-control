"""Small bridge-facing facade over the Sawyer ROS adapter."""

from __future__ import annotations

from .sawyer import SawyerInterface


class Robot:
    def __init__(self):
        self._sawyer = SawyerInterface()

    def move_to_joints(self, angles, timeout: float) -> bool:
        return self._sawyer.move_to_joint_positions(angles, timeout)

    def get_state(self) -> dict:
        return {
            "joint_angles": self._sawyer.joint_positions(),
            "joint_velocities": self._sawyer.joint_velocities(),
            "joint_efforts": self._sawyer.joint_efforts(),
            "endpoint_pose": self._sawyer.endpoint_pose(),
        }
