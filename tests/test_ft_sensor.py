import socket
import struct
import sys
import threading
import time
from pathlib import Path

import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'bridge'))
from robot_api.hardware.ft_sensor import FTSensorManager


class Box:
    def __init__(self):
        self.scale = 100
        self.enabled = True
    def raw(self):
        return dict(cfgcpf=str(self.scale), cfgcpt='10', scfgfu='N', scfgtu='Nm',
                    comrdte='1' if self.enabled else '0')


class Socket:
    def __init__(self, owner):
        self.owner = owner
        self.running = False
        self.seq = 0xfffffffd
        self.closed = False
    def setsockopt(self, *args): pass
    def settimeout(self, *args): pass
    def connect(self, *args): pass
    def send(self, data):
        _, cmd, _ = struct.unpack('!HHI', data)
        self.owner.commands.append(cmd)
        self.running = cmd == 2
    def recv(self, size):
        time.sleep(.002)
        if self.owner.silent or not self.running:
            raise socket.timeout()
        self.seq = (self.seq + 1) & 0xffffffff
        return struct.pack('!IIIiiiiii', self.seq, self.seq, 0, *([100]*6))
    def close(self): self.closed = True


class Factory:
    def __init__(self):
        self.commands, self.sockets = [], []
        self.silent = False
    def __call__(self, *args):
        s = Socket(self)
        self.sockets.append(s)
        return s


def await_data(sensor, timeout=2):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        result = sensor.query(max_age_s=1)
        if result['valid']:
            return result
        time.sleep(.005)
    raise AssertionError(sensor.error)


def test_scaling_wraparound_explicit_zero_and_shutdown():
    f, box = Factory(), Box()
    sensor = FTSensorManager(config=box, host='fake', socket_factory=f, settle_s=0)
    try:
        first = await_data(sensor)
        time.sleep(.015)
        assert sensor.query()['dropped'] == 0
        assert first['mean'] == [1,1,1,10,10,10]
        assert 0x42 not in f.commands and 0x41 not in f.commands
        assert sensor.zero(timeout=1)
        second = await_data(sensor)
        assert first['session_id'] != second['session_id']
        assert f.commands.count(0x42) == 1
    finally:
        sensor.close()
    assert all(s.closed for s in f.sockets)
    assert sensor.latest() is None


def test_config_change_refreshes_scaling_and_marks_cursor_reset():
    f, box = Factory(), Box()
    sensor = FTSensorManager(config=box, host='fake', socket_factory=f, settle_s=0)
    try:
        first = await_data(sensor)
        sensor.change(lambda b: setattr(b, 'scale', 200))
        second = await_data(sensor)
        assert second['mean'][:3] == [0.5]*3
        old = sensor.query(session_id=first['session_id'], after_id=first['next_id'])
        assert not old['valid'] and old['session_changed']
        assert second['config_version'] > first['config_version']
    finally:
        sensor.close()


def test_packet_timeout_reconnects_without_bias():
    f = Factory()
    sensor = FTSensorManager(config=Box(), host='fake', socket_factory=f,
                             settle_s=0, timeout_s=.02)
    try:
        first = await_data(sensor)
        f.silent = True
        time.sleep(.08)
        assert not sensor.query()['valid'] and sensor.latest() is None
        f.silent = False
        second = await_data(sensor)
        assert first['session_id'] != second['session_id']
        assert 0x42 not in f.commands
    finally:
        sensor.close()


def test_invalid_scaling_fails_visibly_without_opening_udp():
    f, box = Factory(), Box()
    box.scale = 0
    sensor = FTSensorManager(config=box, host='fake', socket_factory=f, settle_s=0)
    try:
        deadline = time.monotonic()+1
        while sensor.error == 'connecting' and time.monotonic() < deadline:
            time.sleep(.005)
        assert not sensor.query()['valid']
        assert not f.sockets
    finally:
        sensor.close()


def test_failed_config_write_still_restarts_without_old_samples():
    sensor = FTSensorManager(config=Box(), host='fake', socket_factory=Factory(), settle_s=0)
    try:
        first = await_data(sensor)
        def fail(box):
            raise ValueError('bad setting')
        with pytest.raises(ValueError):
            sensor.change(fail)
        second = await_data(sensor)
        assert second['session_id'] != first['session_id']
    finally:
        sensor.close()
