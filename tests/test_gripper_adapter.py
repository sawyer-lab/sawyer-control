import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock

import pytest


@pytest.fixture
def adapter(monkeypatch):
    rospy = ModuleType('rospy')
    rospy.wait_for_message = Mock(return_value=SimpleNamespace(devices=[
        SimpleNamespace(name='stp_021709TP00448')]))
    rospy.loginfo = Mock()
    messages = ModuleType('intera_core_msgs.msg')
    messages.IONodeStatus = object
    hardware = ModuleType('robot_api.hardware')
    hardware.__path__ = []
    plate_module = ModuleType('robot_api.hardware.clicksmart_plate')
    plate_module.SimpleClickSmartGripper = Mock()
    for name, module in [('rospy', rospy), ('intera_core_msgs.msg', messages),
                         ('robot_api.hardware', hardware),
                         ('robot_api.hardware.clicksmart_plate', plate_module)]:
        monkeypatch.setitem(sys.modules, name, module)
    path = Path(__file__).parents[1] / 'bridge/robot_api/hardware/gripper.py'
    spec = importlib.util.spec_from_file_location('robot_api.hardware.gripper_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.Gripper, plate_module.SimpleClickSmartGripper


def test_fixed_plate_initializes(adapter):
    gripper_type, factory = adapter
    plate = factory.return_value
    plate.list_endpoint_names.return_value = ['A']
    plate.get_ee_signal_value.return_value = None
    plate._node_device_status = SimpleNamespace(tag='down')
    gripper = gripper_type()
    factory.assert_called_once_with('stp_021709TP00448', initialize=True)
    assert gripper.get_state() == {'position': -1.0, 'is_grasping': False, 'state': 'down'}
    plate.set_ee_signal_value.assert_not_called()


def test_commands_report_each_endpoint_acknowledgement(adapter):
    gripper_type, factory = adapter
    plate = factory.return_value
    plate.list_endpoint_names.return_value = ['A', 'B']
    plate.set_ee_signal_value.side_effect = [True, False, True, True]
    gripper = gripper_type('stp_test')
    assert gripper.open() is False
    assert gripper.close() is True
    assert [c.args for c in plate.set_ee_signal_value.call_args_list] == [
        ('grip', True), ('grip', True), ('grip', False), ('grip', False)]
