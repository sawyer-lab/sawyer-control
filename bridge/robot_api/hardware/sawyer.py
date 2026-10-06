"""ROS command and state adapter for the physical Sawyer right arm."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import TypedDict, cast

import rospy
from intera_core_msgs.msg import EndpointState, JointCommand
from sensor_msgs.msg import JointState

from .ft_buffer import CLOCK_ID, SampleBuffer


class JointStateSample(TypedDict):
    positions: list[float]
    velocities: list[float]
    efforts: list[float]
    received_ns: int
    sample_id: int
    wall_s: float
    source_stamp_s: float
    velocity_valid: bool


@dataclass
class RobotCommand:
    position: list[float] = field(default_factory=list)
    velocity: list[float] = field(default_factory=list)
    acceleration: list[float] = field(default_factory=list)
    effort: list[float] = field(default_factory=list)


class ControlMode:
    POSITION = JointCommand.POSITION_MODE
    VELOCITY = JointCommand.VELOCITY_MODE
    TORQUE = JointCommand.TORQUE_MODE
    TRAJECTORY = JointCommand.TRAJECTORY_MODE


class SawyerInterface:
    _joint_names = tuple(f"right_j{index}" for index in range(7))

    def __init__(self):
        self._joint_state: JointStateSample | None = None
        self._state_lock = threading.Lock()
        self._publish_lock = threading.Lock()
        self.history = SampleBuffer(10000)
        self._endpoint_pose = None
        self._command = JointCommand(names=self._joint_names)
        self._publisher = rospy.Publisher(
            "/robot/limb/right/joint_command", JointCommand, tcp_nodelay=True, queue_size=1
        )
        rospy.Subscriber(
            "/robot/joint_states", JointState, self._on_joint_state, queue_size=1, tcp_nodelay=True
        )
        rospy.Subscriber(
            "/robot/limb/right/endpoint_state",
            EndpointState,
            self._on_endpoint_state,
            queue_size=1,
            tcp_nodelay=True,
        )
        rospy.loginfo("Waiting for Sawyer joint state")
        deadline = rospy.Time.now() + rospy.Duration(10.0)
        rate = rospy.Rate(100)

        while self._joint_state is None and not rospy.is_shutdown() and rospy.Time.now() < deadline:
            rate.sleep()

        if self._joint_state is None:
            raise RuntimeError("Timed out waiting for Sawyer joint state")

    def _on_joint_state(self, message: JointState) -> None:
        received_ns = time.monotonic_ns()
        values = dict(zip(message.name, message.position))

        if not all(name in values for name in self._joint_names):
            return

        velocities = dict(zip(message.name, message.velocity))
        efforts = dict(zip(message.name, message.effort))
        state = {
            "positions": [values[name] for name in self._joint_names],
            "velocities": [velocities.get(name, 0.0) for name in self._joint_names],
            "efforts": [efforts.get(name, 0.0) for name in self._joint_names],
            "wall_s": time.time(),
            "source_stamp_s": message.header.stamp.to_sec(),
            "velocity_valid": all(name in velocities for name in self._joint_names),
        }

        with self._state_lock:
            self._joint_state = cast(
                JointStateSample,
                self.history.append(state, received_ns),
            )

    def state_snapshot(self):

        with self._state_lock:
            row = self._joint_state

            if row is None:
                return {}

            return {
                "joint_angles": list(row["positions"]),
                "joint_velocities": list(row["velocities"]),
                "joint_efforts": list(row["efforts"]),
                "received_ns": row["received_ns"],
                "sample_id": row["sample_id"],
                "clock_id": CLOCK_ID,
                "session_id": self.history.session_id,
                "timestamp": row["wall_s"],
                "source_stamp_s": row["source_stamp_s"],
                "velocity_valid": row["velocity_valid"],
                "endpoint_pose": self._endpoint_pose,
            }

    def _on_endpoint_state(self, message: EndpointState) -> None:
        pose = message.pose
        self._endpoint_pose = {
            "position": [pose.position.x, pose.position.y, pose.position.z],
            "orientation": [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ],
        }

    def joint_positions(self) -> list[float]:
        return list(self._joint_state["positions"]) if self._joint_state else [0.0] * 7

    def joint_velocities(self) -> list[float]:
        return list(self._joint_state["velocities"]) if self._joint_state else [0.0] * 7

    def joint_efforts(self) -> list[float]:
        return list(self._joint_state["efforts"]) if self._joint_state else [0.0] * 7

    def endpoint_pose(self) -> dict | None:
        return self._endpoint_pose

    def move_to_joint_positions(self, target: list[float], timeout_s: float) -> bool:
        if len(target) != 7:
            return False

        deadline = rospy.Time.now() + rospy.Duration(timeout_s)
        command = RobotCommand(position=list(target))

        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if (
                max(
                    abs(current - desired)
                    for current, desired in zip(self.joint_positions(), target)
                )
                <= 0.015
            ):
                return True
            if not self.command_joint_values(command, ControlMode.POSITION):
                return False

        return False

    def command_joint_values(self, command: RobotCommand, mode: int) -> bool:
        # A distinct message avoids races between concurrent gRPC callers.
        message = JointCommand(names=list(self._joint_names))
        message.mode = mode

        if rospy.is_shutdown():
            return False

        message.position = command.position
        message.velocity = command.velocity
        message.acceleration = command.acceleration
        message.effort = command.effort
        message.header.stamp = rospy.Time.now()

        with self._publish_lock:
            self._publisher.publish(message)

        return True
