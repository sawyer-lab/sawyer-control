"""Exercise actual callback/history code with only ROS messages replaced."""
import importlib.util
from pathlib import Path
import sys
import threading
import types

from robot_api.hardware.ft_buffer import SampleBuffer


def load_adapter(monkeypatch):
    command_type = type('JointCommand', (), dict(POSITION_MODE=1, VELOCITY_MODE=2,
                        TORQUE_MODE=3, TRAJECTORY_MODE=4))
    rospy = types.ModuleType('rospy')
    msg = types.ModuleType('intera_core_msgs.msg')
    msg.EndpointState, msg.JointCommand = object, command_type
    sensor = types.ModuleType('sensor_msgs.msg')
    sensor.JointState = object
    for name, module in [('rospy', rospy), ('intera_core_msgs.msg', msg),
                         ('sensor_msgs.msg', sensor)]:
        monkeypatch.setitem(sys.modules, name, module)
    path = Path(__file__).resolve().parents[1]/'bridge/robot_api/hardware/sawyer.py'
    name = 'robot_api.hardware._sawyer_timing_test'
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, name, module)
    spec.loader.exec_module(module)
    return module.SawyerInterface


def test_callback_histories_real_updates_and_snapshot(monkeypatch):
    cls = load_adapter(monkeypatch)
    adapter = cls.__new__(cls)
    adapter._state_lock = threading.Lock()
    adapter.history = SampleBuffer()
    adapter._joint_state = adapter._endpoint_pose = None
    message = types.SimpleNamespace(name=cls._joint_names, position=list(range(7)),
        velocity=[], effort=[], header=types.SimpleNamespace(
            stamp=types.SimpleNamespace(to_sec=lambda: 123.0)))
    adapter._on_joint_state(message)
    first = adapter.state_snapshot()
    assert first['sample_id'] == 1 and first['received_ns'] > 0
    assert not first['velocity_valid'] and first['source_stamp_s'] == 123
    message.velocity = [2]*7
    adapter._on_joint_state(message)
    second = adapter.state_snapshot()
    assert second['velocity_valid'] and second['sample_id'] == 2
    assert len(adapter.history.query()['rows']) == 2
    second['joint_angles'][0] = 500
    assert adapter.state_snapshot()['joint_angles'][0] == 0
