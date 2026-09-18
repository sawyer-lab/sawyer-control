from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import grpc

from .types import ControlMode, JointCommandSample, JointVector
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


def _slot(slot: int | None) -> int:
    """-1 asks the bridge for the active configuration."""
    return -1 if slot is None else slot


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


class ForceTorqueConfigClient(_Client):
    """Config plane of the force/torque sensor.

    These settings persist in the device across reboots and affect every other
    client of the sensor. Setters return what the device actually accepted -
    the box quantizes and clamps silently - so render the response, not the
    request. Network settings are deliberately not exposed.
    """

    @classmethod
    def connect(cls, address: str = "127.0.0.1:50051") -> "ForceTorqueConfigClient":
        return cls(Endpoint(address))

    def __init__(self, endpoint: Endpoint | None = None):
        super().__init__(endpoint)
        self._api = control_pb2_grpc.ForceTorqueConfigStub(self._channel)

    # ── reads ────────────────────────────────────────────────────────────────

    def identity(self):
        return self._api.GetIdentity(control_pb2.Empty())

    def scaling(self, slot: int | None = None):
        """Counts-per-unit factors. Re-read after any slot or unit change; they
        differ by unit (1e6 per N, but 1 per lbf)."""
        return self._api.GetScaling(control_pb2.FtSlotRequest(slot=_slot(slot)))

    def device_status(self):
        return self._api.GetDeviceStatus(control_pb2.Empty())

    def configuration(self, slot: int | None = None):
        return self._api.GetConfiguration(control_pb2.FtSlotRequest(slot=_slot(slot)))

    def configurations(self):
        return list(self._api.ListConfigurations(control_pb2.Empty()).configurations)

    def settings(self):
        return self._api.GetSettings(control_pb2.Empty())

    def communications(self):
        return self._api.GetCommunications(control_pb2.Empty())

    def monitor_conditions(self):
        return self._api.GetMonitorConditions(control_pb2.Empty())

    def peaks(self):
        return self._api.GetPeaks(control_pb2.Empty())

    # ── writes ───────────────────────────────────────────────────────────────

    def write_configuration(self, slot: int, **fields):
        """Partial update of one slot. Accepts name, calibration, force_unit,
        torque_unit, distance_unit, angle_unit, user_field_a, user_field_b.
        Units take the manual's menu name ("N", "Nm", "mm") or a code."""
        request = control_pb2.FtWriteConfigurationRequest(slot=slot)
        for key, value in fields.items():
            if value is None:
                continue
            setattr(request, key, value if key == "calibration" else str(value))
        return self._api.WriteConfiguration(request)

    def set_tool_transform(self, slot: int, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0,
                           rx: float = 0.0, ry: float = 0.0, rz: float = 0.0,
                           distance_unit=None, angle_unit=None):
        """Set a slot's tool transform. Pass the units explicitly - slots differ
        (slot 0 is mm, slot 15 defaults to inches)."""
        request = control_pb2.FtToolTransformRequest(
            slot=slot, dx=dx, dy=dy, dz=dz, rx=rx, ry=ry, rz=rz)
        if distance_unit is not None:
            request.distance_unit = str(distance_unit)
        if angle_unit is not None:
            request.angle_unit = str(angle_unit)
        return self._api.SetToolTransform(request)

    def select_configuration(self, slot: int):
        """Activate a slot. Zero-based: one less than the numbering on the
        device's own web pages."""
        return self._api.SelectConfiguration(control_pb2.FtSlotRequest(slot=slot))

    def set_filter(self, code: int):
        """0 disables filtering; 1-12 select a cutoff. Note the codes are not
        monotonic - 9-12 are higher cutoffs than 1-8."""
        return self._api.SetFilter(control_pb2.FtFilterRequest(code=code))

    def set_peak_logging(self, enabled: bool):
        return self._api.SetPeakLogging(control_pb2.FtEnableRequest(enabled=enabled))

    def set_bias_vector(self, gages):
        """Six persisted per-strain-gage offsets. Distinct from
        ForceTorqueClient.zero(), which biases from the current load."""
        return self._api.SetBiasVector(control_pb2.FtBiasRequest(gages=list(gages)))

    def clear_bias(self):
        return self._api.ClearBias(control_pb2.Empty())

    def set_rate_hz(self, rate_hz: int):
        """Output rate. Achievable rates are integer fractions of 7000 and the
        device rounds up, so read rate_hz off the response."""
        return self._api.SetRate(control_pb2.FtRateRequest(rate_hz=rate_hz))

    def set_buffer_records(self, records: int):
        return self._api.SetBufferRecords(control_pb2.FtBufferRequest(records=records))

    def set_rdt_enabled(self, enabled: bool):
        return self._api.SetRdtEnabled(control_pb2.FtEnableRequest(enabled=enabled))

    def set_ethernet_ip_enabled(self, enabled: bool):
        return self._api.SetEthernetIpEnabled(control_pb2.FtEnableRequest(enabled=enabled))

    def set_monitor_condition(self, index: int, axis, comparison: str, counts: int,
                              output_code: int, enabled: bool = True):
        """Hardware threshold evaluated at 7 kHz inside the device. `axis` is
        one of fx fy fz tx ty tz, or None to disable. `counts` is raw counts -
        convert with scaling()."""
        return self._api.SetMonitorCondition(control_pb2.FtMonitorConditionRequest(
            index=index, axis=axis or "", comparison=comparison, counts=counts,
            output_code=output_code, enabled=enabled))

    def set_monitor_conditions_enabled(self, enabled: bool):
        return self._api.SetMonitorConditionsEnabled(
            control_pb2.FtEnableRequest(enabled=enabled))


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

    def read_brio(self):
        return self._api.GetFrame(control_pb2.CameraRequest(camera=control_pb2.BRIO))

    def start_hand(self) -> None:
        SawyerRobotClient._require(self._api.Start(control_pb2.CameraRequest(camera=control_pb2.HAND)))

    def stop_hand(self) -> None:
        SawyerRobotClient._require(self._api.Stop(control_pb2.CameraRequest(camera=control_pb2.HAND)))

    def start_head(self) -> None:
        SawyerRobotClient._require(self._api.Start(control_pb2.CameraRequest(camera=control_pb2.HEAD)))

    def stop_head(self) -> None:
        SawyerRobotClient._require(self._api.Stop(control_pb2.CameraRequest(camera=control_pb2.HEAD)))

    def start_brio(self) -> None:
        SawyerRobotClient._require(self._api.Start(control_pb2.CameraRequest(camera=control_pb2.BRIO)))

    def stop_brio(self) -> None:
        SawyerRobotClient._require(self._api.Stop(control_pb2.CameraRequest(camera=control_pb2.BRIO)))

    def hand_frames(self, rate_hz: float = 15.0):
        return self._api.StreamFrames(control_pb2.CameraStreamRequest(camera=control_pb2.HAND, rate_hz=rate_hz))

    def head_frames(self, rate_hz: float = 15.0):
        return self._api.StreamFrames(control_pb2.CameraStreamRequest(camera=control_pb2.HEAD, rate_hz=rate_hz))

    def brio_frames(self, rate_hz: float = 15.0):
        return self._api.StreamFrames(control_pb2.CameraStreamRequest(camera=control_pb2.BRIO, rate_hz=rate_hz))
