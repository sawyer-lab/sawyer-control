from __future__ import annotations

import logging
import os
import threading
from dataclasses import dataclass, field

from robot_api.hardware.brio import BrioCamera
from robot_api.hardware.camera import Camera
from robot_api.hardware.ft_config import NetBoxConfig
from robot_api.hardware.ft_sensor import FTSensorManager
from robot_api.hardware.gripper import Gripper
from robot_api.hardware.robot import Robot
from robot_api.hardware.robot_enable import RobotEnable
from robot_api.hardware.sawyer import ControlMode, RobotCommand

logger = logging.getLogger(__name__)


def _build_ft_config() -> NetBoxConfig:
    host = os.environ.get("FT_SENSOR_IP", "192.168.1.11")
    if host.lower() == "disabled":
        raise RuntimeError("FT sensor disabled via FT_SENSOR_IP=disabled")
    return NetBoxConfig(host)


def _optional(factory):
    try:
        return factory()
    except (OSError, RuntimeError) as exc:
        logger.warning(
            "Optional component unavailable: %s",
            exc,
            exc_info=True,
        )
        return None


@dataclass
class Runtime:
    robot: Robot
    enable: RobotEnable
    cameras: dict[str, Camera]
    gripper: Gripper | None = None
    ft: FTSensorManager | None = None
    ft_config: NetBoxConfig | None = None
    brio: BrioCamera | None = None
    _ft_initialized: bool = False
    _ft_config_initialized: bool = False
    _hardware_lock: threading.RLock = field(default_factory=threading.RLock)

    @classmethod
    def build(cls) -> Runtime:
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

    def get_force_torque(self) -> FTSensorManager | None:
        if self._ft_initialized:
            return self.ft
        with self._hardware_lock:
            if not self._ft_initialized:
                box = self.get_force_torque_config()
                self.ft = FTSensorManager(config=box) if box is not None else None
                self._ft_initialized = True
            return self.ft

    def get_force_torque_config(self) -> NetBoxConfig | None:
        """Config plane of the force/torque sensor, or None when the sensor is
        disabled. Construction performs no I/O and writes nothing; the bridge
        only reads or writes settings when a client asks it to."""
        with self._hardware_lock:
            if not self._ft_config_initialized:
                self.ft_config = _optional(_build_ft_config)
                self._ft_config_initialized = True

            return self.ft_config

    def change_force_torque(self, action):
        with self._hardware_lock:
            box = self.get_force_torque_config()

            if box is None:
                raise RuntimeError("Force-torque sensor is disabled")

            if self.ft is not None:
                return self.ft.change(action)

            return action(box)

    def close(self):
        with self._hardware_lock:
            if self.ft is not None:
                self.ft.close()

    def state_history(self, **kwargs):
        return self.robot._sawyer.history.query(**kwargs)

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

    def move_to(self, joints: list[float], timeout_s: float) -> bool:
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
