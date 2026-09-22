from pathlib import Path

import pytest

from sawyer_control.experiments.trajectory import resample_trajectory


def _trajectory(path: Path) -> None:
    rows = []
    for index in range(11):
        t = index / 10
        rows.append(','.join([str(t)] + [str(index)] * 7))
    path.write_text('\n'.join(rows) + '\n', encoding='utf-8')


@pytest.mark.parametrize(
    ('speed', 'expected_duration'),
    [(0.75, 1.34), (1.0, 1.0), (1.25, 0.8)],
)
def test_time_scaling_keeps_command_rate_and_endpoint(tmp_path, speed, expected_duration):
    path = tmp_path / 'trajectory.csv'
    _trajectory(path)
    result = resample_trajectory(path, command_rate_hz=100.0, speed_scale=speed)
    assert result.metadata['played_rate_hz'] == 100.0
    assert result.metadata['played_duration_s'] == pytest.approx(expected_duration)
    assert result.positions[0][0] == pytest.approx(0.0)
    assert result.positions[-1][0] == pytest.approx(10.0)
