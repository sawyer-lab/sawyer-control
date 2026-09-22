#!/usr/bin/env python3
"""Compare F/T sensor settings against the same contact-free robot motion.

The bridge owns the ATI RDT stream. This script remains a standalone manual
entry point, while its motion and acquisition implementation lives in
``sawyer_control.experiments`` so calibration campaigns can reuse it directly.
Bias is never changed.
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src'))

from sawyer_control.experiments import (
    SmoothMoveSettings,
    TrajectoryExperimentSettings,
    TrajectoryTrial,
    run_trajectory_experiment,
)

BRIDGE = '127.0.0.1:50051'
TRAJECTORY_CSV = (
    '/home/nexus/Documents/GitHub/contact-implicit-operations/data/replay_results_run060.csv'
)
RATE_HZ = 100.0

APPROACH_RATE_HZ = 100.0
APPROACH_MIN_DURATION_S = 5.0
APPROACH_MAX_SPEED_RAD_S = 0.35
APPROACH_TOLERANCE_RAD = 0.03
APPROACH_HOLD_S = 0.5

SETTLE_S = 1.0
PRE_ROLL_S = 0.2
POST_ROLL_S = 0.2
BETWEEN_S = 1.0

FT_POLL_HZ = 100.0
JOINT_POLL_HZ = 100.0
MAX_AGE_S = 0.1
RPC_TIMEOUT_S = 0.25
COMMAND_TIMEOUT_S = 0.5
FT_QUERY_LIMIT = 35000
JOINT_QUERY_LIMIT = 10000
ACQUISITION_START_TIMEOUT_S = 8.0

TRIALS = [
    {'rate': 500, 'filter': 0},
    # {'rate': 500, 'filter': 1},
    # {'rate': 500, 'filter': 2},
    # {'rate': 500, 'filter': 3},
    # {'rate': 500, 'filter': 4},
    # {'rate': 500, 'filter': 5},
    # {'rate': 500, 'filter': 6},
    # {'rate': 500, 'filter': 7},
    # {'rate': 500, 'filter': 8},
]

RESTORE = True
RESULTS_ROOT = Path(__file__).resolve().parents[2] / 'results'


def run(
    trials: list[dict[str, int]] | None = None,
    csv_path: str | Path | None = None,
    rate_hz: float | None = None,
    restore_after: bool | None = None,
):
    requested = TRIALS if trials is None else trials
    trajectory = TRAJECTORY_CSV if csv_path is None else csv_path
    command_rate = RATE_HZ if rate_hz is None else rate_hz
    restore = RESTORE if restore_after is None else restore_after

    experiment_trials = [
        TrajectoryTrial(
            sensor_rate_hz=int(item['rate']),
            filter_code=int(item['filter']),
            speed_scale=1.0,
        )
        for item in requested
    ]
    settings = TrajectoryExperimentSettings(
        bridge=BRIDGE,
        command_rate_hz=command_rate,
        settle_s=SETTLE_S,
        pre_roll_s=PRE_ROLL_S,
        post_roll_s=POST_ROLL_S,
        between_s=BETWEEN_S,
        ft_poll_hz=FT_POLL_HZ,
        joint_poll_hz=JOINT_POLL_HZ,
        max_age_s=MAX_AGE_S,
        rpc_timeout_s=RPC_TIMEOUT_S,
        command_timeout_s=COMMAND_TIMEOUT_S,
        ft_query_limit=FT_QUERY_LIMIT,
        joint_query_limit=JOINT_QUERY_LIMIT,
        acquisition_start_timeout_s=ACQUISITION_START_TIMEOUT_S,
        plot=True,
        show_plot=False,
        approach=SmoothMoveSettings(
            rate_hz=APPROACH_RATE_HZ,
            min_duration_s=APPROACH_MIN_DURATION_S,
            max_speed_rad_s=APPROACH_MAX_SPEED_RAD_S,
            tolerance_rad=APPROACH_TOLERANCE_RAD,
            hold_s=APPROACH_HOLD_S,
        ),
    )
    return run_trajectory_experiment(
        RESULTS_ROOT,
        trajectory_csv=trajectory,
        trials=experiment_trials,
        settings=settings,
        restore_after=restore,
        prompt=True,
    )


if __name__ == '__main__':
    run()
