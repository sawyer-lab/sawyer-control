#!/usr/bin/env python3
"""
robot_enable — port of intera_sdk RobotEnable to Python 3 standalone.

Ported from:
  intera_sdk/intera_interface/src/intera_interface/robot_enable.py

Uses local intera_dataflow instead of the SDK package.
"""

import rospy
from std_msgs.msg import Bool, Empty
from intera_core_msgs.msg import AssemblyState

from .intera_dataflow import wait_for


class RobotEnable:
    """
    Simple control/status wrapper around robot state.

    enable()  — enable all joints
    disable() — disable all joints
    reset()   — reset faults and disable
    stop()    — emergency stop (like pressing the e-stop button)
    """

    def __init__(self):
        self._state = None
        self._state_sub = rospy.Subscriber(
            'robot/state', AssemblyState, self._state_callback)

        self._enable_pub = rospy.Publisher('robot/set_super_enable', Bool,  queue_size=10)
        self._reset_pub  = rospy.Publisher('robot/set_super_reset',  Empty, queue_size=10)
        self._stop_pub   = rospy.Publisher('robot/set_super_stop',   Empty, queue_size=10)

        wait_for(
            lambda: self._state is not None,
            timeout=10.0,
            timeout_msg="Failed to get robot state on robot/state",
        )
        rospy.loginfo("RobotEnable: initialized")

    def _state_callback(self, msg):
        self._state = msg

    # ── read-only state ───────────────────────────────────────────────────────

    def state(self):
        """Return the last received AssemblyState message."""
        return self._state

    def is_enabled(self):
        return bool(self._state and self._state.enabled)

    def is_stopped(self):
        return bool(self._state and self._state.stopped)

    def is_error(self):
        return bool(self._state and self._state.error)

    # ── control ───────────────────────────────────────────────────────────────

    def enable(self):
        """Enable all joints when the robot is not stopped."""
        if self._state and self._state.stopped:
            rospy.logerr("RobotEnable: reset the stopped robot explicitly before enabling")
            return False
        wait_for(
            test=lambda: self._state and self._state.enabled,
            timeout=5.0,
            timeout_msg="Failed to enable robot",
            body=lambda: self._enable_pub.publish(True),
        )
        rospy.loginfo("RobotEnable: robot enabled")
        return True

    def disable(self):
        """Disable all joints."""
        wait_for(
            test=lambda: self._state and not self._state.enabled,
            timeout=5.0,
            timeout_msg="Failed to disable robot",
            body=lambda: self._enable_pub.publish(False),
        )
        rospy.loginfo("RobotEnable: robot disabled")
        return True

    def reset(self):
        """Reset all joints, clear JRCP faults, disable the robot."""
        if self._state and not self._state.stopped:
            rospy.logwarn("RobotEnable: robot is not in an error state — cannot reset")
            return False
        if self._state and self._state.estop_button == AssemblyState.ESTOP_BUTTON_PRESSED:
            rospy.logerr("RobotEnable: E-Stop is pressed — disengage before reset")
            return False
        rospy.loginfo("RobotEnable: resetting...")
        wait_for(
            test=lambda: (self._state
                          and not self._state.stopped
                          and not self._state.error),
            timeout=5.0,
            timeout_msg="Failed to reset robot",
            body=self._reset_pub.publish,
        )
        rospy.loginfo("RobotEnable: reset complete")
        return True

    def stop(self):
        """Issue a software stop. Robot reset is required before re-enabling."""
        wait_for(
            test=lambda: self._state and self._state.stopped,
            timeout=5.0,
            timeout_msg="Failed to stop robot",
            body=self._stop_pub.publish,
        )
        rospy.loginfo("RobotEnable: robot stopped")
        return True
