import sys
import time
from pathlib import Path
from concurrent import futures

import grpc
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'bridge'))
from robot_api.hardware.ft_buffer import SampleBuffer, CLOCK_ID
from robot_api.hardware.ft_acquisition import summarize
from sawyer_control import ForceTorqueClient, SawyerRobotClient
from sawyer_control.v1 import control_pb2, control_pb2_grpc
from sawyer_control_bridge.service import ForceTorqueService, RobotService


def row(value=1, status=0, dropped=0, version=1):
    return dict(values=[value] * 6, status=status, dropped_before=dropped,
                config_version=version, force_unit='N', torque_unit='Nm',
                rdt_sequence=1, ft_sequence=1, wall_s=time.time())


def test_non_destructive_pagination_and_overflow():
    b = SampleBuffer(3)
    for i in range(5):
        b.append(row(i))
    a = b.query(limit=2)
    assert a['overflow'] and a['truncated'] and a['next_id'] == 4
    assert b.query(limit=2)['rows'] == a['rows']
    tail = b.query(session_id=a['session_id'], after_id=a['next_id'])
    assert not tail['overflow'] and [r['sample_id'] for r in tail['rows']] == [5]


def test_session_restart_and_cursor_require_session():
    b = SampleBuffer()
    b.append(row())
    first = b.query()
    b.reset()
    b.append(row(9))
    batch = b.query(session_id=first['session_id'], after_id=1)
    assert batch['session_changed'] and not batch['rows'] and batch['next_id'] == 0
    with pytest.raises(ValueError):
        b.query(after_id=1)


def test_half_open_window_and_coverage():
    b = SampleBuffer()
    base = time.monotonic_ns()
    b.append(row(2), base)
    b.append(row(4), base + 1)
    b.append(row(8), base + 2)
    time.sleep(.001)
    batch = b.query(start_ns=base, end_ns=base + 2)
    assert summarize(batch)['mean'] == [3] * 6
    assert b.query(start_ns=1)['incomplete_window']


@pytest.mark.parametrize('kind', ['empty', 'fault', 'loss', 'stale', 'mixed', 'reader'])
def test_invalid_data_never_becomes_zero_or_valid_mean(kind):
    b = SampleBuffer()
    if kind != 'empty':
        b.append(row(status=(1 << 17) if kind == 'fault' else 0,
                     dropped=1 if kind == 'loss' else 0), time.monotonic_ns()-200_000_000)
    if kind == 'mixed':
        b.append(row(version=2))
    result = summarize(b.query(), max_age_s=.1 if kind == 'stale' else 1,
                       reader_error='lost connection' if kind == 'reader' else '')
    assert not result['valid'] and result['mean'] == [] and result['reasons']


class Sensor:
    def __init__(self):
        self.buffer = SampleBuffer()
        self.buffer.append(row(6))
    def query(self, max_age_s=.1, **kwargs):
        return dict(summarize(self.buffer.query(**kwargs), max_age_s), reader_error='')


class Runtime:
    def __init__(self):
        self.sensor = Sensor()
        self.joints = SampleBuffer()
        self.joints.append(dict(positions=[1]*7, velocities=[2]*7, efforts=[3]*7,
                                source_stamp_s=5, wall_s=time.time(), velocity_valid=True))
    def get_force_torque(self):
        return self.sensor
    def state_history(self, **kwargs):
        return self.joints.query(**kwargs)
    def command(self, mode, sample):
        return True


@pytest.fixture
def clients():
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=4))
    runtime = Runtime()
    control_pb2_grpc.add_ForceTorqueServicer_to_server(ForceTorqueService(runtime), server)
    control_pb2_grpc.add_RobotControlServicer_to_server(RobotService(runtime), server)
    port = server.add_insecure_port('127.0.0.1:0')
    server.start()
    with ForceTorqueClient.connect(f'127.0.0.1:{port}') as ft, SawyerRobotClient.connect(f'127.0.0.1:{port}') as robot:
        yield ft, robot, runtime
    server.stop(None).wait()


def test_window_round_trip_and_independent_consumers(clients):
    ft, _, _ = clients
    first = ft.window(include_samples=True, max_age_s=1)
    assert first.valid and list(first.mean) == [6]*6
    assert first.history.clock_id == CLOCK_ID and len(first.samples) == 1
    assert ft.window(max_age_s=1).history.next_id == first.history.next_id
    empty = ft.window(session_id=first.history.session_id, after_id=first.history.next_id)
    assert not empty.valid and list(empty.mean) == []


def test_explicit_clock_validation_and_deadline(clients):
    ft, _, _ = clients
    with pytest.raises(grpc.RpcError) as e:
        ft.window(start_ns=1, clock_id='another-host')
    assert e.value.code() == grpc.StatusCode.INVALID_ARGUMENT
    with pytest.raises(grpc.RpcError):
        ft.window(max_age_s=float('nan'))


def test_timed_commands_and_joint_history(clients):
    from sawyer_control import JointCommandSample, ControlMode
    _, robot, _ = clients
    history = robot.state_history()
    assert history.samples[0].received_ns > 0
    assert history.samples[0].velocity_valid
    assert history.samples[0].session_id == history.history.session_id
    ack = robot.command_timed(JointCommandSample(position=[0]*7), ControlMode.POSITION, 'cycle-4')
    assert ack.cycle_id == 'cycle-4' and ack.clock_id == CLOCK_ID
    assert ack.published_ns >= ack.received_ns > 0


def test_diagnostic_demo_runs_without_commanding_motion(tmp_path):
    import os
    import subprocess
    from test_ft_config import FakeBox
    from sawyer_control_bridge.service import ForceTorqueConfigService
    class DemoRuntime(Runtime):
        def get_force_torque_config(self):
            return self.box
        def command(self, *args):
            raise AssertionError('Diagnostic must never command the robot')
    runtime = DemoRuntime()
    runtime.box = FakeBox()
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=4))
    control_pb2_grpc.add_ForceTorqueServicer_to_server(ForceTorqueService(runtime), server)
    control_pb2_grpc.add_RobotControlServicer_to_server(RobotService(runtime), server)
    control_pb2_grpc.add_ForceTorqueConfigServicer_to_server(ForceTorqueConfigService(runtime), server)
    port = server.add_insecure_port('127.0.0.1:0')
    server.start()
    try:
        script = Path(__file__).resolve().parents[1]/'scripts/demo/ft_acquisition.py'
        result = subprocess.run([sys.executable, str(script), '--bridge', f'127.0.0.1:{port}',
            '--duration', '.04', '--max-age-ms', '2000', '--output', str(tmp_path/'log.jsonl')],
            env=os.environ.copy(), capture_output=True, text=True, timeout=10)
        assert result.returncode == 0, result.stderr
        assert 'invalid_queries' in result.stdout
        import json
        rows = [json.loads(line) for line in (tmp_path/'log.jsonl').read_text().splitlines()]
        assert rows[0]['type'] == 'configuration'
        assert rows[-1]['type'] == 'summary'
        assert any(r['type'] == 'cycle' for r in rows)
    finally:
        server.stop(None).wait()


def test_recent_ft_bootstrap_does_not_start_from_zero_cursor():
    from types import SimpleNamespace

    from sawyer_control.experiments.acquisition import _wait_for_recent_ft

    class FullBufferClient:
        def __init__(self):
            self.calls = []

        def window(self, **kwargs):
            self.calls.append(kwargs)
            history = SimpleNamespace(session_id='session-1', newest_id=42000)
            if kwargs.get('window_s'):
                return SimpleNamespace(
                    valid=False,
                    reasons=['no_samples'],
                    reader_error='',
                    history=history,
                )
            assert kwargs['session_id'] == 'session-1'
            assert kwargs['after_id'] == 41999
            return SimpleNamespace(
                valid=True,
                reasons=[],
                reader_error='',
                history=history,
            )

    ft = FullBufferClient()
    result = _wait_for_recent_ft(
        ft,
        max_age_s=0.1,
        timeout_s=0.25,
        start_timeout_s=0.2,
    )

    assert result.valid
    assert result.history.newest_id == 42000
    assert len(ft.calls) == 2
    assert ft.calls[0]['window_s'] == pytest.approx(0.02)
    assert 'after_id' not in ft.calls[0]
    assert ft.calls[1]['after_id'] == 41999
    assert ft.calls[1]['limit'] == 1

