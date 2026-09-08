#!/usr/bin/env python3
"""
io_command — port of intera_sdk intera_io/io_command.py to Python 3 standalone.

IOCommand    : generic IO command container
SetCommand   : specialised set-signal / set-port command builder
"""

import json
import rospy
from intera_core_msgs.msg import IOComponentCommand


class IOCommand:
    """Container for a generic IO command."""

    def __init__(self, op, args=None, time=None, now=False):
        self.time = rospy.Time() if time is None else time
        if now:
            self.time = rospy.Time.now()
        self.op   = op
        self.args = args if args else {}

    def __str__(self):
        return str({"time": self.time, "op": self.op, "args": self.args})

    def as_msg(self, now=None):
        """
        Return a properly formatted IOComponentCommand ROS message.

        @param now: True=set time to now; False=reset to (0,0);
                    None (default)=set to now only if currently unset
        """
        if now is None:
            self.time = self.time if not self.time.is_zero() else rospy.Time.now()
        elif now:
            self.time = rospy.Time.now()
        else:
            self.time = rospy.Time()

        return IOComponentCommand(
            time=self.time,
            op=self.op,
            args=json.dumps(self.args),
        )


class SetCommand(IOCommand):
    """Command builder for set-signal / set-port operations."""

    def __init__(self, args=None):
        super().__init__('set', args)

    def _set(self, components, name, data_type, dimensions, *values):
        self.args.setdefault(components, {})
        entry = {'format': {'type': data_type}, 'data': list(values)}
        if dimensions > 1:
            entry['format']['dimensions'] = [dimensions]
        self.args[components][name] = entry

    def set_signal(self, signal_name, data_type, *signal_value):
        """Add a set-signal command; returns self for chaining."""
        self._set('signals', signal_name, data_type, len(signal_value), *signal_value)
        return self

    def set_port(self, port_name, data_type, *port_value):
        """Add a set-port command; returns self for chaining."""
        self._set('ports', port_name, data_type, len(port_value), *port_value)
        return self
