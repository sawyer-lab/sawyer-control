"""ROS command and state adapter for the physical Sawyer right arm."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional

import rospy
from intera_core_msgs.msg import EndpointState, JointCommand
from sensor_msgs.msg import JointState


@dataclass
class RobotCommand:
    position: List[float] = field(default_factory=list)
    velocity: List[float] = field(default_factory=list)
    acceleration: List[float] = field(default_factory=list)
    effort: List[float] = field(default_factory=list)


class ControlMode:
    POSITION = JointCommand.POSITION_MODE
    VELOCITY = JointCommand.VELOCITY_MODE
    TORQUE = JointCommand.TORQUE_MODE
    TRAJECTORY = JointCommand.TRAJECTORY_MODE


class SawyerInterface:
    _joint_names = [f"right_j{index}" for index in range(7)]

    def __init__(self):
        self._joint_state = None
        self._endpoint_pose = None
        self._command = JointCommand(names=self._joint_names)
        self._publisher = rospy.Publisher(
            "/robot/limb/right/joint_command", JointCommand, tcp_nodelay=True, queue_size=1
        )
        rospy.Subscriber("/robot/joint_states", JointState, self._on_joint_state, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber(
            "/robot/limb/right/endpoint_state", EndpointState, self._on_endpoint_state, queue_size=1, tcp_nodelay=True
        )
        rospy.loginfo("Waiting for Sawyer joint state")
        deadline = rospy.Time.now() + rospy.Duration(10.0)
        rate = rospy.Rate(100)
        while self._joint_state is None and not rospy.is_shutdown() and rospy.Time.now() < deadline:
            rate.sleep()
        if self._joint_state is None:
            raise RuntimeError("Timed out waiting for Sawyer joint state")

    def _on_joint_state(self, message: JointState) -> None:
        values = dict(zip(message.name, message.position))
        if not all(name in values for name in self._joint_names):
            return
        velocities = dict(zip(message.name, message.velocity))
        efforts = dict(zip(message.name, message.effort))
        self._joint_state = {
            "positions": [values[name] for name in self._joint_names],
            "velocities": [velocities.get(name, 0.0) for name in self._joint_names],
            "efforts": [efforts.get(name, 0.0) for name in self._joint_names],
        }

    def _on_endpoint_state(self, message: EndpointState) -> None:
        pose = message.pose
        self._endpoint_pose = {
            "position": [pose.position.x, pose.position.y, pose.position.z],
            "orientation": [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
        }

    def joint_positions(self) -> List[float]:
        return list(self._joint_state["positions"]) if self._joint_state else [0.0] * 7

    def joint_velocities(self) -> List[float]:
        return list(self._joint_state["velocities"]) if self._joint_state else [0.0] * 7

    def joint_efforts(self) -> List[float]:
        return list(self._joint_state["efforts"]) if self._joint_state else [0.0] * 7

    def endpoint_pose(self) -> Optional[dict]:
        return self._endpoint_pose

    def move_to_joint_positions(self, target: List[float], timeout_s: float) -> bool:
        if len(target) != 7:
            return False
        deadline = rospy.Time.now() + rospy.Duration(timeout_s)
        command = RobotCommand(position=list(target))
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if max(abs(current - desired) for current, desired in zip(self.joint_positions(), target)) <= 0.015:
                return True
            if not self.execute_sequence([command], ControlMode.POSITION, 100.0):
                return False
        return False

    def execute_sequence(
        self,
        commands: List[RobotCommand],
        mode: int,
        rate_hz: float,
        cancelled: Optional[Callable[[], bool]] = None,
    ) -> bool:
        if not commands or rate_hz <= 0:
            return False
        rate = rospy.Rate(rate_hz)
        self._command.mode = mode
        for command in commands:
            if rospy.is_shutdown() or (cancelled is not None and cancelled()):
                return False
            self._command.position = command.position
            self._command.velocity = command.velocity
            self._command.acceleration = command.acceleration
            self._command.effort = command.effort
            self._command.header.stamp = rospy.Time.now()
            self._publisher.publish(self._command)
            rate.sleep()
        return True
