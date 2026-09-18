from __future__ import annotations

import time
from concurrent import futures

import grpc

from robot_api.hardware import ft_status
from robot_api.hardware.ft_config import NetBoxError
from sawyer_control.v1 import control_pb2, control_pb2_grpc


def _values(vector):
    return list(vector.values) if vector.values else None


def _sample(sample):
    return {
        "position": _values(sample.position),
        "velocity": _values(sample.velocity),
        "effort": _values(sample.effort),
        "acceleration": _values(sample.acceleration),
    }


def _valid(sample, mode):
    required = {
        control_pb2.POSITION: ("position",),
        control_pb2.VELOCITY: ("velocity",),
        control_pb2.TORQUE: ("effort",),
        control_pb2.TRAJECTORY: ("position", "velocity", "acceleration"),
    }.get(mode)
    if required is None or any(sample.get(field) is None for field in required):
        return False
    return all(values is None or len(values) == 7 for values in sample.values())


class RobotService(control_pb2_grpc.RobotControlServicer):
    def __init__(self, runtime):
        self._runtime = runtime

    def Health(self, request, context):
        return control_pb2.HealthReply(protocol_version="v1", bridge_version="0.1.0")

    def GetState(self, request, context):
        return _robot_state(self._runtime.state())

    def StreamState(self, request, context):
        rate_hz = _stream_rate(request.rate_hz, context, 100.0)
        while context.is_active():
            yield _robot_state(self._runtime.state())
            time.sleep(1.0 / rate_hz)

    def GetPose(self, request, context):
        return _robot_state(self._runtime.state()).pose

    def MoveTo(self, request, context):
        if len(request.target.values) != 7:
            return control_pb2.CommandResult(success=False, message="Expected seven joint values")
        ok = self._runtime.move_to(list(request.target.values), request.timeout_s or 30.0)
        return control_pb2.CommandResult(success=ok, message="" if ok else "Movement failed")

    def CommandJoints(self, request, context):
        sample = _sample(request.sample)
        if not _valid(sample, request.mode):
            return control_pb2.CommandResult(success=False, message="Invalid command for control mode")
        ok = self._runtime.command(request.mode, sample)
        return control_pb2.CommandResult(success=ok, message="" if ok else "Command rejected")

    def Stop(self, request, context):
        ok = self._runtime.stop()
        return control_pb2.CommandResult(success=ok, message="" if ok else "Stop failed")

    def Enable(self, request, context):
        return _result(self._runtime.enable.enable())

    def Disable(self, request, context):
        return _result(self._runtime.enable.disable())

    def Reset(self, request, context):
        return _result(self._runtime.enable.reset())

    def OpenGripper(self, request, context):
        return _result(self._runtime.get_gripper().open())

    def CloseGripper(self, request, context):
        return _result(self._runtime.get_gripper().close())

    def GetGripperState(self, request, context):
        state = self._runtime.get_gripper().get_state()
        return control_pb2.GripperState(position=state.get("position", 0.0), grasping=state.get("is_grasping", False), state=state.get("state", "unknown"))


class ForceTorqueService(control_pb2_grpc.ForceTorqueServicer):
    def __init__(self, runtime):
        self._runtime = runtime

    def GetReading(self, request, context):
        return _ft_reading(self._runtime, context)

    def StreamReadings(self, request, context):
        rate_hz = _stream_rate(request.rate_hz, context, 100.0)
        while context.is_active():
            yield _ft_reading(self._runtime, context)
            time.sleep(1.0 / rate_hz)

    def Zero(self, request, context):
        sensor = self._runtime.get_force_torque()
        if sensor is None:
            context.abort(grpc.StatusCode.UNAVAILABLE, "Force-torque sensor is unavailable")
        return _result(sensor.zero())


class ForceTorqueConfigService(control_pb2_grpc.ForceTorqueConfigServicer):
    """Config plane. Every setter validates in ft_config before issuing a
    request and returns what the device actually accepted, so callers must
    render the response rather than what they asked for."""

    def __init__(self, runtime):
        self._runtime = runtime

    def _box(self, context):
        box = self._runtime.get_force_torque_config()
        if box is None:
            context.abort(grpc.StatusCode.UNAVAILABLE, "Force-torque sensor is unavailable")
        return box

    def _call(self, context, action):
        """Map device and validation failures onto gRPC status codes."""
        try:
            return action(self._box(context))
        except ValueError as exc:
            context.abort(grpc.StatusCode.INVALID_ARGUMENT, str(exc))
        except NetBoxError as exc:
            context.abort(grpc.StatusCode.UNAVAILABLE, str(exc))

    # ── reads ────────────────────────────────────────────────────────────────

    def GetIdentity(self, request, context):
        raw = self._call(context, lambda box: box.identity())
        return control_pb2.FtIdentity(
            host=raw["host"], ip=raw["ip"], mac=raw["mac"], firmware=raw["firmware"],
            serial=raw["serial"], internal_rate_hz=raw["internal_rate_hz"])

    def GetScaling(self, request, context):
        return _ft_scaling(self._call(context, lambda box: box.scaling(_slot(request.slot))))

    def GetDeviceStatus(self, request, context):
        raw = self._call(context, lambda box: box.status())
        status = raw["status"]
        return control_pb2.FtDeviceStatus(
            status=status, conditions_breached=raw["conditions_breached"],
            condition_output=raw["condition_output"], condition_latched=raw["condition_latched"],
            faults=ft_status.faults(status), healthy=ft_status.is_healthy(status),
            saturated=ft_status.saturated(status))

    def GetConfiguration(self, request, context):
        return _ft_configuration(
            self._call(context, lambda box: box.configuration(_slot(request.slot))))

    def ListConfigurations(self, request, context):
        raw = self._call(context, lambda box: box.configurations())
        return control_pb2.FtConfigurationList(
            configurations=[_ft_configuration(config) for config in raw])

    def GetSettings(self, request, context):
        return _ft_settings(self._call(context, lambda box: box.settings()))

    def GetCommunications(self, request, context):
        return _ft_communications(self._call(context, lambda box: box.communications()))

    def GetMonitorConditions(self, request, context):
        conditions = self._call(context, lambda box: box.monitor_conditions())
        enabled = self._call(context, lambda box: box.settings()["monitor_conditions_enabled"])
        return control_pb2.FtMonitorConditions(
            conditions=[_ft_condition(c) for c in conditions], enabled=enabled)

    def GetPeaks(self, request, context):
        raw = self._call(context, lambda box: box.peaks())
        return control_pb2.FtPeaks(min_counts=raw["min_counts"], max_counts=raw["max_counts"],
                                   enabled=raw["enabled"])

    # ── writes ───────────────────────────────────────────────────────────────

    def WriteConfiguration(self, request, context):
        fields = {name: _unit(getattr(request, name))
                  for name in ("name", "force_unit", "torque_unit", "distance_unit",
                               "angle_unit", "user_field_a", "user_field_b")
                  if request.HasField(name)}
        if request.HasField("calibration"):
            fields["calibration"] = request.calibration
        return _ft_configuration(self._call(
            context, lambda box: box.write_configuration(request.slot, **fields)))

    def SetToolTransform(self, request, context):
        units = {name: _unit(getattr(request, name))
                 for name in ("distance_unit", "angle_unit") if request.HasField(name)}

        def action(box):
            box.set_tool_transform(request.slot, request.dx, request.dy, request.dz,
                                   request.rx, request.ry, request.rz, **units)
            return box.configuration(request.slot)

        return _ft_configuration(self._call(context, action))

    def SelectConfiguration(self, request, context):
        def action(box):
            box.select_configuration(request.slot)
            return box.settings()

        return _ft_settings(self._call(context, action))

    def SetFilter(self, request, context):
        return _ft_settings(self._call(
            context, lambda box: (box.set_filter(request.code), box.settings())[1]))

    def SetPeakLogging(self, request, context):
        return _ft_settings(self._call(
            context, lambda box: (box.set_peak_logging(request.enabled), box.settings())[1]))

    def SetBiasVector(self, request, context):
        return _ft_settings(self._call(
            context, lambda box: (box.set_bias_vector(list(request.gages)), box.settings())[1]))

    def ClearBias(self, request, context):
        return _ft_settings(self._call(
            context, lambda box: (box.clear_bias(), box.settings())[1]))

    def SetRate(self, request, context):
        return _ft_communications(self._call(
            context, lambda box: (box.set_rate_hz(request.rate_hz), box.communications())[1]))

    def SetBufferRecords(self, request, context):
        return _ft_communications(self._call(
            context, lambda box: (box.set_buffer_records(request.records),
                                  box.communications())[1]))

    def SetRdtEnabled(self, request, context):
        return _ft_communications(self._call(
            context, lambda box: (box.set_rdt_enabled(request.enabled),
                                  box.communications())[1]))

    def SetEthernetIpEnabled(self, request, context):
        return _ft_communications(self._call(
            context, lambda box: (box.set_ethernet_ip_enabled(request.enabled),
                                  box.communications())[1]))

    def SetMonitorCondition(self, request, context):
        axis = request.axis or None
        return _ft_condition(self._call(context, lambda box: box.set_monitor_condition(
            request.index, axis, request.comparison, request.counts,
            request.output_code, request.enabled)))

    def SetMonitorConditionsEnabled(self, request, context):
        return _ft_settings(self._call(
            context, lambda box: (box.set_monitor_conditions_enabled(request.enabled),
                                  box.settings())[1]))


class CameraService(control_pb2_grpc.CameraServicer):
    def __init__(self, runtime):
        self._runtime = runtime

    def GetFrame(self, request, context):
        return _camera_frame(self._runtime, request.camera, context)

    def Start(self, request, context):
        return _result(_camera(self._runtime, request.camera, context).start())

    def Stop(self, request, context):
        return _result(_camera(self._runtime, request.camera, context).stop())

    def StreamFrames(self, request, context):
        rate_hz = _stream_rate(request.rate_hz, context, 15.0)
        while context.is_active():
            yield _camera_frame(self._runtime, request.camera, context)
            time.sleep(1.0 / rate_hz)


def _result(value):
    return control_pb2.CommandResult(success=bool(value), message="" if value else "Command failed")


def _stream_rate(value, context, default):
    rate_hz = value or default
    if rate_hz <= 0:
        context.abort(grpc.StatusCode.INVALID_ARGUMENT, "rate_hz must be positive")
    return rate_hz


def _slot(slot):
    """Protobuf cannot express "unset" for a plain int; -1 means active slot."""
    return None if slot < 0 else slot


def _unit(value):
    """Units arrive as strings; pass numeric ones through as codes."""
    return int(value) if isinstance(value, str) and value.lstrip("-").isdigit() else value


def _ft_scaling(raw):
    return control_pb2.FtScaling(
        counts_per_force=raw["counts_per_force"], counts_per_torque=raw["counts_per_torque"],
        force_unit=raw["force_unit"], torque_unit=raw["torque_unit"])


def _ft_configuration(raw):
    return control_pb2.FtConfiguration(
        slot=raw["slot"], active_slot=raw["active_slot"], name=raw["name"],
        calibration=raw["calibration"], calibration_serial=raw["calibration_serial"],
        tool_transform=raw["tool_transform"], tool_distance_unit=raw["tool_distance_unit"],
        tool_angle_unit=raw["tool_angle_unit"], force_unit_code=raw["force_unit_code"],
        torque_unit_code=raw["torque_unit_code"],
        tool_distance_unit_code=raw["tool_distance_unit_code"],
        tool_angle_unit_code=raw["tool_angle_unit_code"],
        sensing_ranges=raw["sensing_ranges"], user_field_a=raw["user_field_a"],
        user_field_b=raw["user_field_b"], scaling=_ft_scaling(raw))


def _ft_settings(raw):
    cutoff = raw["filter_cutoff_hz"]
    return control_pb2.FtSettings(
        active_slot=raw["active_slot"], filter_code=raw["filter_code"],
        filter_cutoff_hz=cutoff or 0, filter_enabled=cutoff is not None,
        peak_logging=raw["peak_logging"],
        monitor_conditions_enabled=raw["monitor_conditions_enabled"],
        bias_vector=raw["bias_vector"])


def _ft_communications(raw):
    return control_pb2.FtCommunications(
        rdt_enabled=raw["rdt_enabled"], rate_hz=raw["rate_hz"],
        buffer_records=raw["buffer_records"], internal_rate_hz=raw["internal_rate_hz"])


def _ft_condition(raw):
    return control_pb2.FtMonitorCondition(
        index=raw["index"], enabled=raw["enabled"], axis=raw["axis"] or "",
        axis_code=raw["axis_code"], comparison=raw["comparison"], counts=raw["counts"],
        output_code=raw["output_code"])


def _ft_reading(runtime, context):
    sensor = runtime.get_force_torque()
    if sensor is None:
        context.abort(grpc.StatusCode.UNAVAILABLE, "Force-torque sensor is unavailable")
    reading = sensor.latest()
    if reading is None:
        context.abort(grpc.StatusCode.UNAVAILABLE, "Force-torque sensor has no reading")
    return control_pb2.ForceTorqueReading(
        fx=reading["fx"], fy=reading["fy"], fz=reading["fz"],
        tx=reading["tx"], ty=reading["ty"], tz=reading["tz"],
        sequence=reading["seq"], status=reading["status"], timestamp_s=reading["timestamp"],
        faults=ft_status.faults(reading["status"]),
        healthy=ft_status.is_healthy(reading["status"]),
    )


def _camera_frame(runtime, camera_id, context):
    camera = _camera(runtime, camera_id, context)
    data = camera.get_image_compressed()
    image = camera.get_image()
    if data is None or image is None:
        context.abort(grpc.StatusCode.UNAVAILABLE, "Camera has no frame")
    height, width = image.shape[:2]
    return control_pb2.ImageFrame(data=data, encoding="jpeg", width=width, height=height, timestamp_s=time.time())


def _camera(runtime, camera_id, context):
    camera_name = {control_pb2.HEAD: "head", control_pb2.HAND: "hand"}.get(camera_id)
    if camera_name is not None:
        return runtime.cameras[camera_name]
    if camera_id == control_pb2.BRIO:
        try:
            return runtime.get_brio()
        except RuntimeError as error:
            context.abort(grpc.StatusCode.UNAVAILABLE, str(error))
    context.abort(grpc.StatusCode.INVALID_ARGUMENT, "A head, hand, or Brio camera is required")


def _robot_state(raw):
    pose = raw.get("endpoint_pose") or {}
    position = pose.get("position") or [0.0, 0.0, 0.0]
    orientation = pose.get("orientation") or [0.0, 0.0, 0.0, 1.0]
    return control_pb2.RobotState(
        timestamp_s=float(raw.get("timestamp", time.time())),
        positions=control_pb2.JointVector(values=raw.get("joint_angles", [])),
        velocities=control_pb2.JointVector(values=raw.get("joint_velocities", [])),
        efforts=control_pb2.JointVector(values=raw.get("joint_efforts", [])),
        pose=control_pb2.Pose(
            position=control_pb2.Position(x=position[0], y=position[1], z=position[2]),
            orientation=control_pb2.Quaternion(x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3]),
        ),
        enabled=bool(raw.get("enabled", False)),
        stopped=bool(raw.get("stopped", False)),
    )


def build_server(runtime):
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=16))
    control_pb2_grpc.add_RobotControlServicer_to_server(RobotService(runtime), server)
    control_pb2_grpc.add_ForceTorqueServicer_to_server(ForceTorqueService(runtime), server)
    control_pb2_grpc.add_ForceTorqueConfigServicer_to_server(ForceTorqueConfigService(runtime), server)
    control_pb2_grpc.add_CameraServicer_to_server(CameraService(runtime), server)
    return server
