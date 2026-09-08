from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Iterable

import grpc

from .types import CommandSequence, ControlMode, JointCommandSample, JointVector
from .v1 import control_pb2, control_pb2_grpc

_MODE_TO_PROTO = {
    ControlMode.POSITION: control_pb2.POSITION,
    ControlMode.VELOCITY: control_pb2.VELOCITY,
    ControlMode.TORQUE: control_pb2.TORQUE,
    ControlMode.TRAJECTORY: control_pb2.TRAJECTORY,
}


class BridgeError(RuntimeError):
    pass


@dataclass(frozen=True)
class Endpoint:
    address: str = "127.0.0.1:50051"


class _Client:
    def __init__(self, endpoint: Endpoint | None = None):
        self._channel = grpc.insecure_channel((endpoint or Endpoint()).address)

    def close(self) -> None:
        self._channel.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


def _vector(values: JointVector | Iterable[float]) -> control_pb2.JointVector:
    vector = values if isinstance(values, JointVector) else JointVector(values)
    return control_pb2.JointVector(values=vector.values)


def _sample(value: JointCommandSample) -> control_pb2.JointCommandSample:
    result = control_pb2.JointCommandSample()
    if value.position is not None:
        result.position.CopyFrom(_vector(value.position))
    if value.velocity is not None:
        result.velocity.CopyFrom(_vector(value.velocity))
    if value.effort is not None:
        result.effort.CopyFrom(_vector(value.effort))
    if value.acceleration is not None:
        result.acceleration.CopyFrom(_vector(value.acceleration))
    return result


class SawyerRobotClient(_Client):
    @classmethod
    def connect(cls, address: str = "127.0.0.1:50051") -> "SawyerRobotClient":
        return cls(Endpoint(address))

    def __init__(self, endpoint: Endpoint | None = None):
        super().__init__(endpoint)
        self._api = control_pb2_grpc.RobotControlStub(self._channel)

    def health(self) -> str:
        return self._api.Health(control_pb2.HealthRequest()).protocol_version

    def get_state(self):
        return self._api.GetState(control_pb2.Empty())

    def states(self, rate_hz: float = 100.0):
        return self._api.StreamState(control_pb2.StreamStateRequest(rate_hz=rate_hz))

    def get_pose(self):
        return self._api.GetPose(control_pb2.Empty())

    def move_to(self, target: JointVector | Iterable[float], timeout_s: float = 30.0) -> None:
        self._require(self._api.MoveTo(control_pb2.MoveToRequest(target=_vector(target), timeout_s=timeout_s)))

    def command(self, sample: JointCommandSample, mode: ControlMode) -> None:
        self._require(self._api.CommandJoints(control_pb2.JointCommandRequest(mode=_MODE_TO_PROTO[mode], sample=_sample(sample))))

    def execute_sequence(self, sequence: CommandSequence, mode: ControlMode, rate_hz: float = 100.0) -> str:
        operation = self._api.StartSequence(control_pb2.SequenceRequest(
            mode=_MODE_TO_PROTO[mode], samples=[_sample(sample) for sample in sequence.samples], rate_hz=rate_hz))
        return operation.id

    def wait_for_sequence(self, operation_id: str, timeout_s: float = 30.0) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            status = self._api.GetOperation(control_pb2.OperationRequest(id=operation_id))
            if status.phase == "done":
                return
            if status.phase in {"aborted", "failed", "unknown"}:
                raise BridgeError(status.message or status.phase)
            time.sleep(0.02)
        raise TimeoutError(f"Sequence {operation_id} did not finish in time")

    def stop(self) -> None:
        self._require(self._api.Stop(control_pb2.StopRequest()))

    def enable(self) -> None:
        self._require(self._api.Enable(control_pb2.Empty()))

    def disable(self) -> None:
        self._require(self._api.Disable(control_pb2.Empty()))

    def reset(self) -> None:
        self._require(self._api.Reset(control_pb2.Empty()))

    def open_gripper(self) -> None:
        self._require(self._api.OpenGripper(control_pb2.Empty()))

    def close_gripper(self) -> None:
        self._require(self._api.CloseGripper(control_pb2.Empty()))

    def get_gripper_state(self):
        return self._api.GetGripperState(control_pb2.Empty())

    @staticmethod
    def _require(result) -> None:
        if not result.success:
            raise BridgeError(result.message)


class ForceTorqueClient(_Client):
    @classmethod
    def connect(cls, address: str = "127.0.0.1:50051") -> "ForceTorqueClient":
        return cls(Endpoint(address))

    def __init__(self, endpoint: Endpoint | None = None):
        super().__init__(endpoint)
        self._api = control_pb2_grpc.ForceTorqueStub(self._channel)

    def read(self):
        return self._api.GetReading(control_pb2.Empty())

    def zero(self) -> None:
        SawyerRobotClient._require(self._api.Zero(control_pb2.Empty()))

    def readings(self, rate_hz: float = 100.0):
        return self._api.StreamReadings(control_pb2.StreamReadingsRequest(rate_hz=rate_hz))


class CameraClient(_Client):
    @classmethod
    def connect(cls, address: str = "127.0.0.1:50051") -> "CameraClient":
        return cls(Endpoint(address))

    def __init__(self, endpoint: Endpoint | None = None):
        super().__init__(endpoint)
        self._api = control_pb2_grpc.CameraStub(self._channel)

    def read_hand(self):
        return self._api.GetFrame(control_pb2.CameraRequest(camera=control_pb2.HAND))

    def read_head(self):
        return self._api.GetFrame(control_pb2.CameraRequest(camera=control_pb2.HEAD))

    def hand_frames(self, rate_hz: float = 15.0):
        return self._api.StreamFrames(control_pb2.CameraStreamRequest(camera=control_pb2.HAND, rate_hz=rate_hz))

    def head_frames(self, rate_hz: float = 15.0):
        return self._api.StreamFrames(control_pb2.CameraStreamRequest(camera=control_pb2.HEAD, rate_hz=rate_hz))
