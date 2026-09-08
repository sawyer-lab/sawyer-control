from __future__ import annotations

import threading
import time
import uuid
from concurrent import futures
from dataclasses import dataclass
from typing import Dict, Optional

import grpc

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


@dataclass
class _Operation:
    phase: str = "running"
    message: str = ""
    cancelled: bool = False


class SequenceExecutor:
    def __init__(self, runtime):
        self._runtime = runtime
        self._lock = threading.Lock()
        self._operations: Dict[str, _Operation] = {}
        self._active = None

    def start(self, mode, samples, rate_hz):
        with self._lock:
            if self._active is not None:
                raise RuntimeError("A sequence is already running")
            operation_id = str(uuid.uuid4())
            operation = _Operation()
            self._operations[operation_id] = operation
            self._active = operation_id
        thread = threading.Thread(target=self._run, args=(operation_id, mode, samples, rate_hz), daemon=True)
        thread.start()
        return operation_id

    def _run(self, operation_id, mode, samples, rate_hz):
        with self._lock:
            operation = self._operations[operation_id]
        try:
            ok = self._runtime.execute(mode, samples, rate_hz, lambda: operation.cancelled)
            with self._lock:
                operation.phase = "aborted" if operation.cancelled else ("done" if ok else "failed")
                operation.message = "cancelled" if operation.cancelled else ("" if ok else "robot rejected sequence")
        except Exception as exc:
            with self._lock:
                operation.phase = "failed"
                operation.message = str(exc)
        finally:
            with self._lock:
                self._active = None

    def stop(self):
        with self._lock:
            if self._active is not None:
                self._operations[self._active].cancelled = True
        return self._runtime.stop()

    def status(self, operation_id):
        with self._lock:
            return self._operations.get(operation_id)


class RobotService(control_pb2_grpc.RobotControlServicer):
    def __init__(self, runtime):
        self._runtime = runtime
        self._sequences = SequenceExecutor(runtime)

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

    def StartSequence(self, request, context):
        samples = [_sample(sample) for sample in request.samples]
        if request.rate_hz <= 0 or not samples or not all(_valid(sample, request.mode) for sample in samples):
            context.abort(grpc.StatusCode.INVALID_ARGUMENT, "Invalid sequence")
        try:
            return control_pb2.Operation(id=self._sequences.start(request.mode, samples, request.rate_hz))
        except RuntimeError as exc:
            context.abort(grpc.StatusCode.FAILED_PRECONDITION, str(exc))

    def GetOperation(self, request, context):
        operation = self._sequences.status(request.id)
        if operation is None:
            return control_pb2.OperationStatus(id=request.id, phase="unknown", message="Unknown operation")
        return control_pb2.OperationStatus(id=request.id, phase=operation.phase, message=operation.message)

    def Stop(self, request, context):
        ok = self._sequences.stop()
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
    )


def _camera_frame(runtime, camera_id, context):
    camera = _camera(runtime, camera_id, context)
    data = camera.get_image_compressed()
    image = camera.get_image()
    if data is None or image is None:
        context.abort(grpc.StatusCode.UNAVAILABLE, f"{camera_name} camera has no frame")
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
    control_pb2_grpc.add_CameraServicer_to_server(CameraService(runtime), server)
    return server
