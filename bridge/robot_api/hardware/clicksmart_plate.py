#!/usr/bin/env python3
"""
clicksmart_plate — port of intera_sdk SimpleClickSmartGripper to Python 3 standalone.

Ported from:
  intera_sdk/intera_interface/src/intera_interface/clicksmart_plate.py

Uses local io_command / io_interface / intera_dataflow modules instead of
the SDK packages (which have Python-2-style imports incompatible with
our container's Python 3 ROS environment).
"""

import json

import rospy

from intera_core_msgs.msg import IONodeStatus, IOComponentCommand

from .intera_dataflow import wait_for
from .io_command import IOCommand
from .io_interface import IODeviceInterface


class SimpleClickSmartGripper:
    """
    Interface for a ClickSmart SmartToolPlate on the Intera Research Robot.

    Uses EE Signal Types (grip, open, closed, object_kg …) to control
    and query the device. The endpoint_id can be omitted when the device
    has a single endpoint.
    """

    def __init__(self, ee_device_id, initialize=True):
        self.name                = ee_device_id
        self.endpoint_map        = None
        self._node_state         = None
        self._node_device_status = None

        self._node_cmd_pub  = rospy.Publisher(
            'io/end_effector/command', IOComponentCommand, queue_size=10)
        self._node_state_sub = rospy.Subscriber(
            'io/end_effector/state', IONodeStatus, self._node_state_cb)

        # Wait until we can see our device in the node state
        wait_for(
            lambda: self._node_device_status is not None,
            timeout=5.0,
            raise_on_error=initialize,
            timeout_msg="Failed to get gripper — no gripper attached?",
        )

        self.gripper_io = IODeviceInterface("end_effector", self.name)
        self.gripper_io.config_changed.connect(self._load_endpoint_info)
        if self.gripper_io.is_config_valid():
            self._load_endpoint_info()

        if initialize and self.needs_init():
            self.initialize()

    # ── ROS callbacks ─────────────────────────────────────────────────────────

    def _node_state_cb(self, msg):
        if not self._node_state or IODeviceInterface._time_changed(
                self._node_state.time, msg.time):
            self._node_state = msg
            # track our device's status
            if msg.devices and msg.devices[0].name == self.name:
                self._node_device_status = msg.devices[0].status
            else:
                self._node_device_status = None

    def _load_endpoint_info(self):
        device_config  = json.loads(self.gripper_io.config.device.config)
        self.endpoint_map = device_config['params']['endpoints']

    # ── state ─────────────────────────────────────────────────────────────────

    def is_ready(self):
        """True if the device is activated, responsive, and has valid state."""
        return (self._node_device_status is not None
                and self._node_device_status.tag == 'ready'
                and self.gripper_io.is_valid())

    def needs_init(self):
        """True if the device is down or unready (needs activate)."""
        return (self._node_device_status is not None
                and self._node_device_status.tag in ('down', 'unready'))

    # ── initialisation ────────────────────────────────────────────────────────

    def initialize(self, timeout=5.0):
        """
        Activate the ClickSmart device (brings tag from 'down' → 'ready').
        May cause slight arm motion due to mass-compensation update.
        """
        rospy.loginfo("ClickSmart: activating...")
        cmd = IOCommand('activate', {"devices": [self.name]})
        self._node_cmd_pub.publish(cmd.as_msg())
        if timeout:
            wait_for(
                self.is_ready,
                timeout=timeout,
                timeout_msg="Failed to initialize ClickSmart gripper",
            )

    # ── EE Signal API ─────────────────────────────────────────────────────────

    def get_ee_signal_value(self, ee_signal_type, endpoint_id=None):
        """
        Return the current value of the given EE Signal Type.

        @param ee_signal_type: e.g. 'grip', 'open', 'closed', 'object_kg'
        @param endpoint_id:    optional; defaults to first endpoint
        """
        _, ep_info = self.get_endpoint_info(endpoint_id)
        if ee_signal_type in ep_info:
            return self.gripper_io.get_signal_value(ep_info[ee_signal_type])

    def set_ee_signal_value(self, ee_signal_type, value, endpoint_id=None, timeout=5.0):
        """
        Set the value of the given EE Signal Type.

        @param ee_signal_type: e.g. 'grip'
        @param value:          bool or float
        @param endpoint_id:    optional; defaults to first endpoint
        """
        _, ep_info = self.get_endpoint_info(endpoint_id)
        if ee_signal_type in ep_info:
            self.gripper_io.set_signal_value(ep_info[ee_signal_type], value)

    def get_ee_signals(self, endpoint_id=None):
        """
        Return dict of {EE Signal Type → signal_name} for the given endpoint.
        The keys are the type strings to pass to get/set_ee_signal_value().
        """
        _, ep_info = self.get_endpoint_info(endpoint_id)
        exclude = {'endpoint_id', 'label', 'type', 'actuationTimeS'}
        return {k: v for k, v in ep_info.items() if k not in exclude}

    def get_all_ee_signals(self):
        """Return nested dict {endpoint_id → {EE Signal Type → signal_name}}."""
        return {ep: self.get_ee_signals(ep) for ep in self.list_endpoint_names()}

    def get_all_signals(self):
        """Return raw signal dict {signal_name → {type, data, role}} from IODeviceInterface."""
        return self.gripper_io.signals

    # ── endpoint helpers ──────────────────────────────────────────────────────

    def list_endpoint_names(self):
        """Return list of endpoint_ids configured on this device."""
        return list(self.endpoint_map.keys()) if self.endpoint_map else []

    def get_endpoint_info(self, endpoint_id=None):
        """
        Return (endpoint_id, endpoint_info) for the default or specified endpoint.
        """
        if not self.endpoint_map:
            raise RuntimeError("No endpoint map loaded — device may not be configured")
        eid = list(self.endpoint_map.keys())[0] if endpoint_id is None else endpoint_id
        return eid, self.endpoint_map[eid]

    def send_configuration(self, config, timeout=5.0):
        """Send a new JSON EndEffector configuration to the device."""
        rospy.loginfo("ClickSmart: reconfiguring...")
        cmd = IOCommand('reconfigure',
                        {"devices": {self.name: config}, "write_config": True})
        self._node_cmd_pub.publish(cmd.as_msg())
        if timeout:
            delay = 3.0
            deadline = rospy.Time.now() + rospy.Duration(delay)
            wait_for(
                lambda: (rospy.Time.now() > deadline
                         and (self.is_ready() or self.needs_init())),
                timeout=max(timeout, delay),
                body=lambda: self._node_cmd_pub.publish(cmd.as_msg()),
                timeout_msg="Failed to reconfigure ClickSmart gripper",
            )

    def __getattr__(self, name):
        """Proxy unknown attributes to the underlying IODeviceInterface."""
        return getattr(self.gripper_io, name)
