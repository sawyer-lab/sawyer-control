#!/usr/bin/env python3
"""
intera_dataflow — minimal port of the Intera SDK dataflow utilities.

Ported from intera_sdk/intera_interface/src/intera_dataflow/ to Python 3
standalone (no intera_sdk package dependency).
"""

import errno
import inspect
from weakref import WeakKeyDictionary, WeakSet

import rospy


# ── Signal ────────────────────────────────────────────────────────────────────

class Signal:
    """Lightweight signal/slot mechanism (Qt-style)."""

    def __init__(self):
        self._functions = WeakSet()
        self._methods   = WeakKeyDictionary()

    def __call__(self, *args, **kwargs):
        for f in self._functions:
            f(*args, **kwargs)
        for obj, funcs in list(self._methods.items()):
            for f in funcs:
                f(obj, *args, **kwargs)

    def connect(self, slot):
        if inspect.ismethod(slot):
            self._methods.setdefault(slot.__self__, set()).add(slot.__func__)
        else:
            self._functions.add(slot)

    def disconnect(self, slot):
        if inspect.ismethod(slot):
            if slot.__self__ in self._methods:
                self._methods[slot.__self__].discard(slot.__func__)
        else:
            self._functions.discard(slot)


# ── wait_for ──────────────────────────────────────────────────────────────────

def wait_for(test, timeout=10.0, raise_on_error=True, rate=100,
             timeout_msg="timeout expired", body=None):
    """
    Spin until *test()* returns truthy or timeout expires.

    @param test:           zero-arg callable returning bool
    @param timeout:        max seconds to wait; negative / inf = indefinite
    @param raise_on_error: raise OSError on timeout/shutdown, or just return False
    @param rate:           polling rate in Hz
    @param timeout_msg:    message for the timeout exception
    @param body:           optional callable to run each iteration while waiting
    @return: True if test passed, False if timed out (when raise_on_error=False)
    """
    end_time   = rospy.get_time() + timeout
    rate_obj   = rospy.Rate(rate)
    no_timeout = (timeout < 0.0) or timeout == float("inf")

    while not test():
        if rospy.is_shutdown():
            if raise_on_error:
                raise OSError(errno.ESHUTDOWN, "ROS shutdown")
            return False
        if (not no_timeout) and rospy.get_time() >= end_time:
            if raise_on_error:
                raise OSError(errno.ETIMEDOUT, timeout_msg)
            return False
        if callable(body):
            body()
        rate_obj.sleep()

    return True
