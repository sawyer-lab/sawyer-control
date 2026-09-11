from concurrent import futures

import grpc
import pytest

from sawyer_control import CameraClient, ControlMode, ForceTorqueClient, JointCommandSample, SawyerRobotClient
from sawyer_control.v1 import control_pb2, control_pb2_grpc


class _Robot(control_pb2_grpc.RobotControlServicer):
    def Health(self, request, context):
        return control_pb2.HealthReply(protocol_version="v1")

    def MoveTo(self, request, context):
        assert list(request.target.values) == [0.0] * 7
        return control_pb2.CommandResult(success=True)

    def CommandJoints(self, request, context):
        assert request.mode == control_pb2.VELOCITY
        assert list(request.sample.velocity.values) == [0.1] * 7
        return control_pb2.CommandResult(success=True)


class _ForceTorque(control_pb2_grpc.ForceTorqueServicer):
    def GetReading(self, request, context):
        return control_pb2.ForceTorqueReading(fz=3.5)

    def Zero(self, request, context):
        return control_pb2.CommandResult(success=True)


class _Camera(control_pb2_grpc.CameraServicer):
    def Start(self, request, context):
        return control_pb2.CommandResult(success=True)

    def Stop(self, request, context):
        return control_pb2.CommandResult(success=True)

    def GetFrame(self, request, context):
        return control_pb2.ImageFrame(data=b"frame", encoding="jpeg", width=2, height=1)


@pytest.fixture
def address():
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=3))
    control_pb2_grpc.add_RobotControlServicer_to_server(_Robot(), server)
    control_pb2_grpc.add_ForceTorqueServicer_to_server(_ForceTorque(), server)
    control_pb2_grpc.add_CameraServicer_to_server(_Camera(), server)
    port = server.add_insecure_port("127.0.0.1:0")
    server.start()
    yield "127.0.0.1:%s" % port
    server.stop(grace=0)


def test_flat_robot_client(address):
    with SawyerRobotClient.connect(address) as robot:
        assert robot.health() == "v1"
        robot.move_to([0.0] * 7)
        robot.command(JointCommandSample(velocity=[0.1] * 7), ControlMode.VELOCITY)


def test_sensor_clients_are_independent(address):
    with ForceTorqueClient.connect(address) as force_torque:
        assert force_torque.read().fz == 3.5
        force_torque.zero()
    with CameraClient.connect(address) as camera:
        camera.start_hand()
        assert camera.read_hand().data == b"frame"
        assert camera.read_head().width == 2
        camera.stop_hand()
        camera.start_brio()
        assert camera.read_brio().height == 1
        camera.stop_brio()
