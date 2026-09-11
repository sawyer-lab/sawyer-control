from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import Dict, List, Optional

from robot_api.hardware.camera import Camera
from robot_api.hardware.brio import BrioCamera
from robot_api.hardware.ft_sensor import FTSensorManager
from robot_api.hardware.gripper import Gripper
from robot_api.hardware.robot import Robot
from robot_api.hardware.robot_enable import RobotEnable
from robot_api.hardware.sawyer import ControlMode, RobotCommand


def _optional(factory):
    try:
        return factory()
    except Exception:
        return None


@dataclass
class Runtime:
    robot: Robot
    enable: RobotEnable
    cameras: Dict[str, Camera]
    gripper: Optional[Gripper] = None
    ft: Optional[FTSensorManager] = None
    brio: Optional[BrioCamera] = None
    _ft_initialized: bool = False
    _hardware_lock: threading.Lock = field(default_factory=threading.Lock)

    @classmethod
    def build(cls) -> "Runtime":
        runtime = cls(
            robot=Robot(),
            enable=RobotEnable(),
            cameras={"head": Camera("head_camera"), "hand": Camera("right_hand_camera")},
        )
        runtime.gripper = Gripper()
        return runtime

    def get_gripper(self) -> Gripper:
        with self._hardware_lock:
            if self.gripper is None:
                self.gripper = Gripper()
            return self.gripper

    def get_force_torque(self) -> Optional[FTSensorManager]:
        with self._hardware_lock:
            if not self._ft_initialized:
                self.ft = _optional(FTSensorManager)
                self._ft_initialized = True
            return self.ft

    def get_brio(self) -> BrioCamera:
        with self._hardware_lock:
            if self.brio is None:
                self.brio = BrioCamera()
            return self.brio

    def state(self) -> dict:
        state = self.robot.get_state()
        state["enabled"] = self.enable.is_enabled()
        state["stopped"] = self.enable.is_stopped()
        return state

    def move_to(self, joints: List[float], timeout_s: float) -> bool:
        return bool(self.robot.move_to_joints(joints, timeout=timeout_s))

    def command(self, mode: int, sample: dict) -> bool:
        control_mode = {
            1: ControlMode.POSITION,
            2: ControlMode.VELOCITY,
            3: ControlMode.TORQUE,
            4: ControlMode.TRAJECTORY,
        }[mode]
        command = RobotCommand(
            position=sample.get("position") or [],
            velocity=sample.get("velocity") or [],
            effort=sample.get("effort") or [],
            acceleration=sample.get("acceleration") or [],
        )
        return bool(self.robot._sawyer.command_joint_values(command, control_mode))

    def stop(self) -> bool:
        return bool(self.enable.stop())
