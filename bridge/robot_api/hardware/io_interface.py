#!/usr/bin/env python3
"""
io_interface — port of intera_sdk intera_io/io_interface.py to Python 3 standalone.

IOInterface       : base class for IO topic wrappers
IODeviceInterface : device-level config/state/command interface
"""

import copy
import json
import threading
import uuid
from threading import Lock

import rospy

from intera_core_msgs.msg import (
    IODeviceConfiguration,
    IODeviceStatus,
    IOComponentCommand,
)

from .intera_dataflow import Signal, wait_for
from .io_command import SetCommand


class IOInterface:
    """Base class that wraps the config/state/command topic triple for an IO path."""

    def __init__(self, path_root, config_msg_type, status_msg_type):
        self._path       = path_root
        self.config_mutex = Lock()
        self.state_mutex  = Lock()
        self.cmd_times    = []
        self.ports        = {}
        self.signals      = {}
        self.config       = config_msg_type()
        self.state        = status_msg_type()
        self.config_changed = Signal()
        self.state_changed  = Signal()

        self._config_sub = rospy.Subscriber(
            self._path + "/config", config_msg_type, self.handle_config)
        self._state_sub  = rospy.Subscriber(
            self._path + "/state",  status_msg_type, self.handle_state)
        self._command_pub = rospy.Publisher(
            self._path + "/command", IOComponentCommand, queue_size=10)

        wait_for(
            lambda: self.config is not None and self.is_config_valid(),
            timeout=5.0,
            timeout_msg=f"Failed to get config at {self._path}/config",
        )
        wait_for(
            lambda: self.state is not None and self.is_state_valid(),
            timeout=5.0,
            raise_on_error=False,
        )

    # ── validity ──────────────────────────────────────────────────────────────

    def is_config_valid(self):
        return self.config.time.secs != 0

    def is_state_valid(self):
        return self.state.time.secs != 0

    def is_valid(self):
        return self.is_config_valid() and self.is_state_valid()

    def invalidate_config(self):
        with self.config_mutex:
            self.config.time.secs = 0

    def invalidate_state(self):
        with self.state_mutex:
            self.state.time.secs = 0

    def revalidate(self, timeout, invalidate_state=True, invalidate_config=True):
        if invalidate_state:
            self.invalidate_state()
        if invalidate_config:
            self.invalidate_config()
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        while not self.is_state_valid() and not rospy.is_shutdown():
            rospy.sleep(0.1)
            if rospy.Time.now() > deadline:
                rospy.logwarn("Timed out waiting for IO interface to become valid")
                return False
        return True

    # ── callbacks ─────────────────────────────────────────────────────────────

    def handle_config(self, msg):
        if not self.config or self._time_changed(self.config.time, msg.time):
            with self.config_mutex:
                self.config = msg
                self.config_changed()

    def handle_state(self, msg):
        if not self.state or self._time_changed(self.state.time, msg.time):
            with self.state_mutex:
                self.state = msg
                self.state_changed()
                self._load_state(self.ports,   self.state.ports)
                self._load_state(self.signals, self.state.signals)

    @staticmethod
    def _load_state(store, items):
        for item in items:
            fmt  = json.loads(item.format)
            data = json.loads(item.data)
            store[item.name] = {
                'type': fmt.get('type'),
                'role': fmt.get('role'),
                'data': data[0] if data else None,
            }

    @staticmethod
    def _time_changed(t1, t2):
        return (t1.secs != t2.secs) or (t1.nsecs != t2.nsecs)

    # ── command publishing ────────────────────────────────────────────────────

    def publish_command(self, op, args, timeout=2.0):
        """
        Publish a command and wait for acknowledgement.
        Returns True if acknowledged within timeout.
        """
        cmd_time = rospy.Time.now()
        self.cmd_times.append(cmd_time)
        self.cmd_times = self.cmd_times[-100:]
        cmd_msg = IOComponentCommand(time=cmd_time, op=op, args=json.dumps(args))

        if timeout is not None:
            deadline = rospy.Time.now() + rospy.Duration(timeout)
            while not rospy.is_shutdown():
                self._command_pub.publish(cmd_msg)
                if self.is_state_valid() and cmd_time in self.state.commands:
                    return True
                rospy.sleep(0.1)
                if rospy.Time.now() > deadline:
                    rospy.logwarn("Timed out waiting for command acknowledgement")
                    return False
        return True


class IODeviceInterface(IOInterface):
    """IO interface for a specific device (config + state + command topics)."""

    def __init__(self, node_name, dev_name):
        super().__init__(
            f'io/{node_name}/{dev_name}',
            IODeviceConfiguration,
            IODeviceStatus,
        )
        self._threads            = {}
        self._callback_items     = {}
        self._callback_functions = {}

    # ── signals ───────────────────────────────────────────────────────────────

    def list_signal_names(self):
        with self.state_mutex:
            return copy.deepcopy(list(self.signals.keys()))

    def get_signal_type(self, signal_name):
        with self.state_mutex:
            return copy.deepcopy(self.signals[signal_name]['type']) \
                if signal_name in self.signals else None

    def get_signal_value(self, signal_name):
        with self.state_mutex:
            return copy.deepcopy(self.signals[signal_name]['data']) \
                if signal_name in self.signals else None

    def set_signal_value(self, signal_name, signal_value, signal_type=None, timeout=5.0):
        """Set signal value. Infers type automatically if not provided."""
        if signal_name not in self.list_signal_names():
            rospy.logerr(f"Signal '{signal_name}' not found in {self._path}")
            return
        s_type = signal_type or self.get_signal_type(signal_name)
        if s_type is None:
            rospy.logerr(f"Cannot determine type for signal '{signal_name}'")
            return
        cmd = SetCommand().set_signal(signal_name, s_type, signal_value)
        self.publish_command(cmd.op, cmd.args, timeout=timeout)
        self.revalidate(timeout, invalidate_state=False, invalidate_config=False)

    # ── ports ─────────────────────────────────────────────────────────────────

    def list_port_names(self):
        with self.state_mutex:
            return copy.deepcopy(list(self.ports.keys()))

    def get_port_type(self, port_name):
        with self.state_mutex:
            return copy.deepcopy(self.ports[port_name]['type']) \
                if port_name in self.ports else None

    def get_port_value(self, port_name):
        with self.state_mutex:
            return copy.deepcopy(self.ports[port_name]['data']) \
                if port_name in self.ports else None

    def set_port_value(self, port_name, port_value, port_type=None, timeout=5.0):
        if port_name not in self.list_port_names():
            rospy.logerr(f"Port '{port_name}' not found in {self._path}")
            return
        p_type = port_type or self.get_port_type(port_name)
        if p_type is None:
            rospy.logerr(f"Cannot determine type for port '{port_name}'")
            return
        cmd = SetCommand().set_port(port_name, p_type, port_value)
        self.publish_command(cmd.op, cmd.args, timeout=timeout)
        self.revalidate(timeout, invalidate_state=False, invalidate_config=False)

    # ── callbacks ─────────────────────────────────────────────────────────────

    def register_callback(self, callback_fn, signal_name, poll_rate=10):
        """
        Poll signal_name at poll_rate Hz and call callback_fn when the value changes.
        Returns a callback_id string, or empty string if signal not found.
        """
        if signal_name not in self.list_signal_names():
            return ''
        cb_id = uuid.uuid4()
        sig   = Signal()
        self._callback_items[cb_id]     = sig
        self._callback_functions[cb_id] = callback_fn
        sig.connect(callback_fn)

        def _spin():
            old = self.get_signal_value(signal_name)
            rate = rospy.Rate(poll_rate)
            while not rospy.is_shutdown():
                new = self.get_signal_value(signal_name)
                if new != old:
                    self._callback_items[cb_id](new)
                old = new
                rate.sleep()

        t = threading.Thread(target=_spin, daemon=True)
        t.start()
        self._threads[cb_id] = t
        return cb_id

    def deregister_callback(self, callback_id):
        if callback_id in self._threads:
            self._callback_items[callback_id].disconnect(
                self._callback_functions[callback_id])
            return True
        return False
