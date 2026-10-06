"""
Title       : Sawyer Experimental Acquisition
Module      : sawyer_control.experiments.acquisition
Description : Reusable static and trajectory acquisition through the Sawyer
              gRPC bridge. The module never changes F/T bias unless explicitly
              requested by acquire_static().

Author      : Oscar Federico García Castro
Email       : oscar.garciac@cinvestav.mx
Affiliation : CINVESTAV, Robotics
Advisor     : Dr. Gustavo Arechavaleta

Created     : 2026-10-03
Updated     : 2026-10-03
Version     : 0.1.0
Python      : 3.10+
"""

from __future__ import annotations

import json
import math
import statistics
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

import grpc

from sawyer_control import (
    ControlMode,
    ForceTorqueClient,
    ForceTorqueConfigClient,
    JointCommandSample,
    SawyerRobotClient,
)
from sawyer_control.client import BridgeError
from sawyer_control.experiments.common import (
    next_session_dir,
    proto_dict,
    slugify,
    write_json,
    write_jsonl,
)
from sawyer_control.experiments.motion import SmoothMoveSettings, smooth_joint_move
from sawyer_control.experiments.trajectory import ResampledTrajectory, resample_trajectory


@dataclass(frozen=True)
class StaticAcquisitionSettings:
    bridge: str = "127.0.0.1:50051"
    duration_s: float = 15.0
    poll_rate_hz: float = 100.0
    window_ms: float = 0.0
    max_age_ms: float = 100.0
    timeout_s: float = 0.25
    require_joint_state: bool = False


@dataclass(frozen=True)
class TrajectoryTrial:
    sensor_rate_hz: int
    filter_code: int
    speed_scale: float = 1.0
    name: str | None = None


@dataclass(frozen=True)
class TrajectoryExperimentSettings:
    bridge: str = "127.0.0.1:50051"
    command_rate_hz: float = 100.0
    settle_s: float = 1.0
    pre_roll_s: float = 0.2
    post_roll_s: float = 0.2
    between_s: float = 1.0
    ft_poll_hz: float = 100.0
    joint_poll_hz: float = 100.0
    max_age_s: float = 0.1
    rpc_timeout_s: float = 0.25
    command_timeout_s: float = 0.5
    ft_query_limit: int = 35000
    joint_query_limit: int = 10000
    acquisition_start_timeout_s: float = 8.0
    plot: bool = False
    show_plot: bool = False
    approach: SmoothMoveSettings = SmoothMoveSettings()


def configure_sensor(
    bridge: str,
    *,
    sensor_rate_hz: int,
    filter_code: int,
) -> dict[str, Any]:
    """Apply rate/filter only when needed. F/T bias is never changed."""
    with ForceTorqueConfigClient.connect(bridge) as cfg:
        before_settings = cfg.settings()
        before_comms = cfg.communications()
        if int(before_comms.rate_hz) != int(sensor_rate_hz):
            cfg.set_rate_hz(int(sensor_rate_hz))
        settings = cfg.settings()
        if int(settings.filter_code) != int(filter_code):
            cfg.set_filter(int(filter_code))
        after_settings = cfg.settings()
        after_comms = cfg.communications()
        return {
            "before_settings": proto_dict(before_settings),
            "before_communications": proto_dict(before_comms),
            "effective_settings": proto_dict(after_settings),
            "effective_communications": proto_dict(after_comms),
            "configuration": proto_dict(cfg.configuration()),
        }


def sensor_snapshot(bridge: str) -> dict[str, Any]:
    """Read the current sensor configuration without changing it."""
    with ForceTorqueConfigClient.connect(bridge) as cfg:
        return {
            "settings": proto_dict(cfg.settings()),
            "communications": proto_dict(cfg.communications()),
            "configuration": proto_dict(cfg.configuration()),
            "identity": proto_dict(cfg.identity()),
        }


def _validate_static_settings(settings: StaticAcquisitionSettings) -> None:
    for name, value in (
        ("duration_s", settings.duration_s),
        ("poll_rate_hz", settings.poll_rate_hz),
        ("max_age_ms", settings.max_age_ms),
        ("timeout_s", settings.timeout_s),
    ):
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be finite and positive")
    if not math.isfinite(settings.window_ms) or settings.window_ms < 0:
        raise ValueError("window_ms must be finite and nonnegative")
    if settings.poll_rate_hz > 1000:
        raise ValueError("poll_rate_hz must be <= 1000")


def acquire_static(
    output: Path,
    *,
    settings: StaticAcquisitionSettings = StaticAcquisitionSettings(),
    sensor_rate_hz: int | None = None,
    filter_code: int | None = None,
    zero: bool = False,
    restore_after: bool = True,
) -> dict[str, Any]:
    """Acquire a static JSONL record compatible with ft_approximation."""
    _validate_static_settings(settings)
    output = output.expanduser()

    with ForceTorqueClient.connect(settings.bridge) as ft, ForceTorqueConfigClient.connect(
        settings.bridge
    ) as cfg, SawyerRobotClient.connect(settings.bridge) as robot:
        original_settings = cfg.settings()
        original_comms = cfg.communications()
        changed = sensor_rate_hz is not None or filter_code is not None or zero

        if zero and len(original_settings.bias_vector) != 6:
            raise RuntimeError("Cannot snapshot bias; zero was not issued")

        try:
            if sensor_rate_hz is not None and int(original_comms.rate_hz) != int(sensor_rate_hz):
                cfg.set_rate_hz(int(sensor_rate_hz))
            current_settings = cfg.settings()
            if filter_code is not None and int(current_settings.filter_code) != int(filter_code):
                cfg.set_filter(int(filter_code))
            if zero:
                ft.zero()

            output.parent.mkdir(parents=True, exist_ok=True)
            effective_settings = cfg.settings()
            effective_comms = cfg.communications()

            with output.open("w", encoding="utf-8") as handle:

                def record(value: dict[str, Any]) -> None:
                    handle.write(json.dumps(value, allow_nan=False) + "\n")

                record(
                    {
                        "type": "configuration",
                        "requested": {
                            "sensor_rate_hz": sensor_rate_hz,
                            "filter_code": filter_code,
                            "zero": zero,
                        },
                        "original_settings": proto_dict(original_settings),
                        "original_communications": proto_dict(original_comms),
                        "effective_settings": proto_dict(effective_settings),
                        "effective_communications": proto_dict(effective_comms),
                        "configuration": proto_dict(cfg.configuration()),
                        "clock_note": "bridge monotonic reception, not physical measurement",
                    }
                )

                initial = _wait_for_recent_ft(
                    ft,
                    max_age_s=settings.max_age_ms / 1000,
                    timeout_s=settings.timeout_s,
                    start_timeout_s=8.0,
                )

                # Start at the current tail of the bridge history. The bootstrap
                # query is intentionally a recent time window, not after_id=0,
                # so a long-running/full ring buffer cannot report overflow.
                session = initial.history.session_id
                cursor = initial.history.newest_id
                initial_joint = None
                joint_session = None
                joint_cursor = None
                try:
                    initial_joint = robot.get_state()
                except grpc.RpcError:
                    if settings.require_joint_state:
                        raise
                if initial_joint is not None:
                    if not initial_joint.session_id or initial_joint.sample_id <= 0:
                        if settings.require_joint_state:
                            raise RuntimeError("Robot state does not expose a valid history cursor")
                    else:
                        joint_session = initial_joint.session_id
                        joint_cursor = initial_joint.sample_id
                        record({"type": "initial_joint_state", "state": proto_dict(initial_joint)})

                period = 1.0 / settings.poll_rate_hz
                start = time.monotonic()
                cycle = 0
                valid = invalid = missed = duplicate_windows = 0
                ft_errors = joint_errors = 0
                ft_rtts: list[float] = []
                joint_rtts: list[float] = []
                ages: list[float] = []
                counts: list[int] = []
                joint_counts: list[int] = []
                intervals: list[float] = []
                last_end = None

                while time.monotonic() - start < settings.duration_s:
                    cycle_deadline = start + cycle * period
                    delay = cycle_deadline - time.monotonic()
                    if delay > 0:
                        time.sleep(delay)
                    began = time.monotonic()

                    options: dict[str, Any] = {
                        "max_age_s": settings.max_age_ms / 1000,
                        "include_samples": True,
                        "limit": 35000,
                        "timeout": settings.timeout_s,
                    }
                    if settings.window_ms:
                        options.update(window_s=settings.window_ms / 1000)
                    else:
                        options.update(session_id=session, after_id=cursor)

                    window = None
                    joints = None
                    ft_rtt_ms = joint_rtt_ms = None
                    ft_error = joint_error = None
                    duplicate = False

                    ft_started = time.monotonic()
                    try:
                        window = ft.window(**options)
                        ft_rtt_ms = (time.monotonic() - ft_started) * 1000
                        ft_rtts.append(ft_rtt_ms)
                        if window.history.session_changed or window.history.cursor_ahead:
                            session, cursor = window.history.session_id, 0
                        else:
                            session, cursor = window.history.session_id, window.history.next_id
                        duplicate = bool(settings.window_ms and last_end == window.history.end_ns)
                        duplicate_windows += int(duplicate)
                        last_end = window.history.end_ns
                        valid += int(window.valid)
                        invalid += int(not window.valid)
                        if window.count:
                            ages.append(window.age_s * 1000)
                            counts.append(window.count)
                    except grpc.RpcError as exc:
                        ft_errors += 1
                        ft_error = {"code": exc.code().name, "message": exc.details()}

                    if joint_session is not None and joint_cursor is not None:
                        joint_started = time.monotonic()
                        try:
                            joints = robot.state_history(
                                session_id=joint_session,
                                after_id=joint_cursor,
                                limit=1000,
                                timeout=settings.timeout_s,
                            )
                            joint_rtt_ms = (time.monotonic() - joint_started) * 1000
                            joint_rtts.append(joint_rtt_ms)
                            joint_session = joints.history.session_id
                            joint_cursor = (
                                0 if joints.history.session_changed else joints.history.next_id
                            )
                            joint_counts.append(len(joints.samples))
                        except grpc.RpcError as exc:
                            joint_errors += 1
                            joint_error = {"code": exc.code().name, "message": exc.details()}

                    cycle_record: dict[str, Any] = {
                        "type": "cycle",
                        "cycle_id": cycle,
                        "local_started_s": began,
                        "local_lateness_ms": (began - cycle_deadline) * 1000,
                        "ft_rpc_ms": ft_rtt_ms,
                        "joint_rpc_ms": joint_rtt_ms,
                        "duplicate_window": duplicate,
                    }
                    if window is not None:
                        cycle_record["ft"] = proto_dict(window)
                    if joints is not None:
                        cycle_record["joints"] = proto_dict(joints)
                    if ft_error is not None:
                        cycle_record["ft_error"] = ft_error
                    if joint_error is not None:
                        cycle_record["joint_error"] = joint_error
                    record(cycle_record)

                    intervals.append((time.monotonic() - began) * 1000)
                    cycle += 1
                    due = math.floor((time.monotonic() - start) / period)
                    if due > cycle:
                        missed += due - cycle
                        cycle = due

                def metric(values: Sequence[float]) -> dict[str, float] | None:
                    if not values:
                        return None
                    return {"mean": statistics.mean(values), "maximum": max(values)}

                report: dict[str, Any] = {
                    "type": "summary",
                    "valid_queries": valid,
                    "invalid_queries": invalid,
                    "rpc_errors": ft_errors + joint_errors,
                    "ft_rpc_errors": ft_errors,
                    "joint_rpc_errors": joint_errors,
                    "missed_deadlines": missed,
                    "duplicate_windows": duplicate_windows,
                    "ft_rpc_ms": metric(ft_rtts),
                    "joint_rpc_ms": metric(joint_rtts),
                    "data_age_ms": metric(ages),
                    "samples_per_query": metric(counts),
                    "joint_samples_per_query": metric(joint_counts),
                    "query_cycle_ms": metric(intervals),
                }
                record(report)
                return report
        finally:
            if changed and restore_after:
                failures: list[str] = []
                actions = []
                if sensor_rate_hz is not None:
                    actions.append(lambda: cfg.set_rate_hz(original_comms.rate_hz))
                if filter_code is not None:
                    actions.append(lambda: cfg.set_filter(original_settings.filter_code))
                if zero:
                    actions.append(lambda: cfg.set_bias_vector(original_settings.bias_vector))
                for action in actions:
                    try:
                        action()
                    except grpc.RpcError as exc:
                        failures.append(f"{exc.code().name}: {exc.details()}")
                if failures:
                    raise RuntimeError("Settings restoration failed: " + "; ".join(failures))


def _wait_for_recent_ft(
    ft: ForceTorqueClient,
    *,
    max_age_s: float,
    timeout_s: float,
    start_timeout_s: float,
    window_s: float = 0.02,
    limit: int = 1000,
) -> Any:
    """Return a valid recent F/T sample without replaying old history.

    A new acquisition must not bootstrap from ``after_id=0`` because a
    long-running ring buffer correctly reports ``overflow`` once early samples
    have been overwritten.  First query a short recent time window to obtain
    the current history metadata.  If that aligned window is empty/incomplete
    (for example a sensor that is currently idle), use its ``newest_id`` to
    request only the current tail sample.
    """
    deadline = time.monotonic() + start_timeout_s
    latest = None
    last_rpc_error = None

    while time.monotonic() < deadline:
        try:
            latest = ft.window(
                window_s=window_s,
                max_age_s=max_age_s,
                limit=limit,
                timeout=timeout_s,
            )
            last_rpc_error = None

            if latest.valid and latest.history.session_id and latest.history.newest_id > 0:
                return latest

            history = latest.history
            if history.session_id and history.newest_id > 0:
                tail = ft.window(
                    session_id=history.session_id,
                    after_id=max(0, history.newest_id - 1),
                    max_age_s=max_age_s,
                    limit=1,
                    timeout=timeout_s,
                )
                latest = tail
                if tail.valid and tail.history.session_id and tail.history.newest_id > 0:
                    return tail

        except grpc.RpcError as exc:
            if exc.code() in (grpc.StatusCode.DEADLINE_EXCEEDED, grpc.StatusCode.UNAVAILABLE):
                last_rpc_error = f"{exc.code().name}: {exc.details()}"
                time.sleep(0.05)
                continue
            raise

        time.sleep(0.05)

    reasons = list(latest.reasons) if latest is not None else []
    reader_error = latest.reader_error if latest is not None else "no response"
    raise RuntimeError(
        "No valid initial acquisition: "
        f"reasons={reasons}, reader_error={reader_error}, last_rpc_error={last_rpc_error}"
    )


def _describe_effective(comms: Any, settings: Any) -> str:
    filter_text = f"{settings.filter_cutoff_hz} Hz LPF" if settings.filter_enabled else "no filter"
    return f"{comms.rate_hz} Hz, {filter_text}"


def _apply_sensor_trial(cfg: ForceTorqueConfigClient, trial: TrajectoryTrial) -> tuple[Any, Any]:
    comms = cfg.communications()
    if int(comms.rate_hz) != int(trial.sensor_rate_hz):
        cfg.set_rate_hz(int(trial.sensor_rate_hz))
    settings = cfg.settings()
    if int(settings.filter_code) != int(trial.filter_code):
        cfg.set_filter(int(trial.filter_code))
    return cfg.communications(), cfg.settings()


def _wait_for_valid_ft(
    ft: ForceTorqueClient,
    *,
    settings: TrajectoryExperimentSettings,
) -> Any:
    deadline = time.monotonic() + settings.acquisition_start_timeout_s
    latest = None
    last_rpc_error = None
    while time.monotonic() < deadline:
        try:
            latest = ft.window(
                window_s=0.02,
                max_age_s=settings.max_age_s,
                limit=1000,
                timeout=settings.rpc_timeout_s,
            )
            last_rpc_error = None
        except grpc.RpcError as exc:
            if exc.code() in (grpc.StatusCode.DEADLINE_EXCEEDED, grpc.StatusCode.UNAVAILABLE):
                last_rpc_error = f"{exc.code().name}: {exc.details()}"
                time.sleep(0.05)
                continue
            raise
        if latest.valid and latest.history.session_id:
            return latest
        time.sleep(0.05)
    reasons = list(latest.reasons) if latest is not None else []
    reader_error = latest.reader_error if latest is not None else "no response"
    raise RuntimeError(
        "No valid F/T acquisition: "
        f"reasons={reasons}, reader_error={reader_error}, last_rpc_error={last_rpc_error}"
    )


def _latest_joint_cursor(robot: SawyerRobotClient) -> Any:
    state = robot.get_state()
    if not state.session_id:
        raise RuntimeError("No joint-state history session is available")
    if not state.clock_id:
        raise RuntimeError("Joint state does not contain a bridge clock_id")
    if state.sample_id <= 0:
        raise RuntimeError("Joint state does not contain a valid sample_id")
    return state


class _ForceTorqueCapture(threading.Thread):
    def __init__(
        self,
        bridge: str,
        session_id: str,
        cursor: int,
        clock_id: str,
        settings: TrajectoryExperimentSettings,
    ) -> None:
        super().__init__(daemon=True)
        self.bridge = bridge
        self.session_id = session_id
        self.cursor = cursor
        self.clock_id = clock_id
        self.settings = settings
        self.samples: list[dict[str, Any]] = []
        self.queries: list[dict[str, Any]] = []
        self.rpc_errors: list[dict[str, Any]] = []
        self.fatal_error: str | None = None
        self._stop_event = threading.Event()

    def _query_once(self, ft: ForceTorqueClient) -> None:
        fixed_end_ns = 0
        while True:
            began_ns = time.monotonic_ns()
            try:
                result = ft.window(
                    session_id=self.session_id,
                    after_id=self.cursor,
                    end_ns=fixed_end_ns,
                    clock_id=self.clock_id,
                    max_age_s=self.settings.max_age_s,
                    include_samples=True,
                    limit=self.settings.ft_query_limit,
                    timeout=self.settings.rpc_timeout_s,
                )
            except grpc.RpcError as exc:
                self.rpc_errors.append(
                    {"local_ns": began_ns, "code": exc.code().name, "message": exc.details()}
                )
                return

            self.queries.append(
                {
                    "local_started_ns": began_ns,
                    "local_finished_ns": time.monotonic_ns(),
                    "valid": result.valid,
                    "reasons": list(result.reasons),
                    "count": result.count,
                    "age_s": result.age_s,
                    "first_ns": result.first_ns,
                    "last_ns": result.last_ns,
                    "status": result.status,
                    "dropped": result.dropped,
                    "config_version": result.config_version,
                    "force_unit": result.force_unit,
                    "torque_unit": result.torque_unit,
                    "reader_error": result.reader_error,
                    "history": proto_dict(result.history),
                }
            )
            if result.history.session_changed or result.history.cursor_ahead:
                self.fatal_error = (
                    "F/T session/cursor changed during the experimental pass: "
                    f"{proto_dict(result.history)}"
                )
                return
            self.samples.extend(proto_dict(sample) for sample in result.samples)
            self.cursor = result.history.next_id
            if not result.history.truncated:
                return
            if fixed_end_ns == 0:
                fixed_end_ns = result.history.end_ns

    def run(self) -> None:
        period_s = 1.0 / self.settings.ft_poll_hz
        with ForceTorqueClient.connect(self.bridge) as ft:
            while self.fatal_error is None:
                self._query_once(ft)
                if self.fatal_error is not None:
                    break
                if self._stop_event.wait(period_s):
                    self._query_once(ft)
                    break

    def stop(self) -> None:
        self._stop_event.set()
        self.join(timeout=max(2.0, 4.0 * self.settings.rpc_timeout_s))
        if self.is_alive():
            raise RuntimeError("F/T capture thread did not stop")


class _JointCapture(threading.Thread):
    def __init__(
        self,
        bridge: str,
        session_id: str,
        cursor: int,
        clock_id: str,
        settings: TrajectoryExperimentSettings,
    ) -> None:
        super().__init__(daemon=True)
        self.bridge = bridge
        self.session_id = session_id
        self.cursor = cursor
        self.clock_id = clock_id
        self.settings = settings
        self.samples: list[dict[str, Any]] = []
        self.queries: list[dict[str, Any]] = []
        self.rpc_errors: list[dict[str, Any]] = []
        self.fatal_error: str | None = None
        self._stop_event = threading.Event()

    def _query_once(self, robot: SawyerRobotClient) -> None:
        fixed_end_ns = 0
        while True:
            began_ns = time.monotonic_ns()
            try:
                result = robot.state_history(
                    session_id=self.session_id,
                    after_id=self.cursor,
                    end_ns=fixed_end_ns,
                    clock_id=self.clock_id,
                    limit=self.settings.joint_query_limit,
                    timeout=self.settings.rpc_timeout_s,
                )
            except grpc.RpcError as exc:
                self.rpc_errors.append(
                    {"local_ns": began_ns, "code": exc.code().name, "message": exc.details()}
                )
                return
            self.queries.append(
                {
                    "local_started_ns": began_ns,
                    "local_finished_ns": time.monotonic_ns(),
                    "history": proto_dict(result.history),
                    "sample_count": len(result.samples),
                }
            )
            if result.history.session_changed or result.history.cursor_ahead:
                self.fatal_error = (
                    "Joint-state session/cursor changed during the experimental pass: "
                    f"{proto_dict(result.history)}"
                )
                return
            self.samples.extend(proto_dict(sample) for sample in result.samples)
            self.cursor = result.history.next_id
            if not result.history.truncated:
                return
            if fixed_end_ns == 0:
                fixed_end_ns = result.history.end_ns

    def run(self) -> None:
        period_s = 1.0 / self.settings.joint_poll_hz
        with SawyerRobotClient.connect(self.bridge) as robot:
            while self.fatal_error is None:
                self._query_once(robot)
                if self.fatal_error is not None:
                    break
                if self._stop_event.wait(period_s):
                    self._query_once(robot)
                    break

    def stop(self) -> None:
        self._stop_event.set()
        self.join(timeout=max(2.0, 4.0 * self.settings.rpc_timeout_s))
        if self.is_alive():
            raise RuntimeError("Joint capture thread did not stop")


def _play(
    robot: SawyerRobotClient,
    rows: Sequence[Sequence[float]],
    *,
    rate_hz: float,
    trial_index: int,
    commands: list[dict[str, Any]],
    timeout_s: float,
) -> None:
    period_ns = round(1_000_000_000 / rate_hz)
    start_ns = time.monotonic_ns()
    for index, position in enumerate(rows):
        deadline_ns = start_ns + index * period_ns
        delay_s = (deadline_ns - time.monotonic_ns()) / 1e9
        if delay_s > 0:
            time.sleep(delay_s)
        local_call_ns = time.monotonic_ns()
        lateness_ms = (local_call_ns - deadline_ns) / 1e6
        cycle_id = f"trial-{trial_index:02d}-cycle-{index:06d}"
        try:
            result = robot.command_timed(
                JointCommandSample(position=position),
                ControlMode.POSITION,
                cycle_id=cycle_id,
                timeout=timeout_s,
            )
        except grpc.RpcError as exc:
            commands.append(
                {
                    "cycle_id": cycle_id,
                    "index": index,
                    "reference_position": list(position),
                    "local_deadline_ns": deadline_ns,
                    "local_call_ns": local_call_ns,
                    "local_lateness_ms": lateness_ms,
                    "rpc_error": {"code": exc.code().name, "message": exc.details()},
                }
            )
            raise RuntimeError(
                f"command_timed failed at {cycle_id}: {exc.code().name}: {exc.details()}"
            ) from exc
        commands.append(
            {
                "cycle_id": cycle_id,
                "index": index,
                "reference_position": list(position),
                "local_deadline_ns": deadline_ns,
                "local_call_ns": local_call_ns,
                "local_return_ns": time.monotonic_ns(),
                "local_lateness_ms": lateness_ms,
                "bridge": proto_dict(result),
            }
        )


def _sample_rate_hz(samples: Sequence[dict[str, Any]]) -> float | None:
    if len(samples) < 2:
        return None
    first = int(samples[0].get("received_ns", 0))
    last = int(samples[-1].get("received_ns", 0))
    elapsed_s = (last - first) / 1e9
    return (len(samples) - 1) / elapsed_s if elapsed_s > 0 else None


def _capture_summary(
    ft_capture: _ForceTorqueCapture,
    joint_capture: _JointCapture,
    commands: Sequence[dict[str, Any]],
) -> dict[str, Any]:
    dropped = sum(int(query.get("dropped", 0)) for query in ft_capture.queries)
    invalid_queries = sum(not query.get("valid", False) for query in ft_capture.queries)
    worst_ms = max((row.get("local_lateness_ms", 0.0) for row in commands), default=0.0)
    return {
        "ft_samples": len(ft_capture.samples),
        "ft_received_rate_hz": _sample_rate_hz(ft_capture.samples),
        "ft_queries": len(ft_capture.queries),
        "ft_invalid_queries": invalid_queries,
        "ft_dropped_reported": dropped,
        "ft_rpc_errors": len(ft_capture.rpc_errors),
        "ft_fatal_error": ft_capture.fatal_error,
        "joint_samples": len(joint_capture.samples),
        "joint_queries": len(joint_capture.queries),
        "joint_rpc_errors": len(joint_capture.rpc_errors),
        "joint_fatal_error": joint_capture.fatal_error,
        "commands": len(commands),
        "command_loop_worst_lateness_ms": worst_ms,
    }


def _save_trial(
    trial_dir: Path,
    metadata: dict[str, Any],
    ft_capture: _ForceTorqueCapture,
    joint_capture: _JointCapture,
    commands: Sequence[dict[str, Any]],
) -> None:
    trial_dir.mkdir(parents=True, exist_ok=False)
    write_json(trial_dir / "metadata.json", metadata)
    write_jsonl(trial_dir / "ft_samples.jsonl", ft_capture.samples)
    write_jsonl(
        trial_dir / "ft_queries.jsonl",
        ft_capture.queries + [{"type": "rpc_error", **row} for row in ft_capture.rpc_errors],
    )
    write_jsonl(trial_dir / "joint_states.jsonl", joint_capture.samples)
    write_jsonl(
        trial_dir / "joint_queries.jsonl",
        joint_capture.queries + [{"type": "rpc_error", **row} for row in joint_capture.rpc_errors],
    )
    write_jsonl(trial_dir / "commands.jsonl", commands)


def _plot_trial(
    ft_samples: Sequence[dict[str, Any]],
    *,
    title: str,
    path: Path,
    show: bool,
) -> None:
    if not ft_samples:
        return
    import matplotlib

    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    first_ns = int(ft_samples[0]["received_ns"])
    times = [(int(sample["received_ns"]) - first_ns) / 1e9 for sample in ft_samples]
    values = [sample.get("values", []) for sample in ft_samples]
    if not values or any(len(row) < 6 for row in values):
        return

    axes = ("Fx", "Fy", "Fz", "Tx", "Ty", "Tz")
    force_unit = ft_samples[0].get("force_unit", "")
    torque_unit = ft_samples[0].get("torque_unit", "")
    units = [force_unit] * 3 + [torque_unit] * 3
    figure, panels = plt.subplots(3, 2, figsize=(13, 8), sharex=True)
    for axis_index, (name, unit) in enumerate(zip(axes, units)):
        panel = panels[axis_index % 3][axis_index // 3]
        panel.plot(times, [row[axis_index] for row in values], linewidth=0.6)
        panel.set_title(name, fontsize=9)
        panel.set_ylabel(unit, fontsize=8)
        panel.grid(alpha=0.3)
    for panel in panels[2]:
        panel.set_xlabel("bridge received time from first sample (s)", fontsize=8)
    figure.suptitle(title, fontsize=11)
    figure.tight_layout(rect=(0, 0, 1, 0.96))
    figure.savefig(path, dpi=130)
    if show:
        plt.show()
    plt.close(figure)


def _restore_config(cfg: ForceTorqueConfigClient, settings: Any, comms: Any) -> list[str]:
    failures: list[str] = []
    for action in (
        lambda: cfg.set_rate_hz(comms.rate_hz),
        lambda: cfg.set_filter(settings.filter_code),
    ):
        try:
            action()
        except grpc.RpcError as exc:
            failures.append(f"{exc.code().name}: {exc.details()}")
    return failures


def run_trajectory_experiment(
    output_root: Path,
    *,
    trajectory_csv: Path | str,
    trials: Sequence[TrajectoryTrial],
    settings: TrajectoryExperimentSettings = TrajectoryExperimentSettings(),
    session_dir: Path | None = None,
    restore_after: bool = True,
    prompt: bool = False,
) -> tuple[Path, list[dict[str, Any]]]:
    """Execute trajectory trials and write ft_dynamics-compatible files."""
    if not trials:
        raise ValueError("At least one trajectory trial is required")
    if settings.command_rate_hz <= 0:
        raise ValueError("command_rate_hz must be positive")

    source = Path(trajectory_csv).expanduser()
    cache: dict[float, ResampledTrajectory] = {}
    for trial in trials:
        if trial.speed_scale not in cache:
            cache[trial.speed_scale] = resample_trajectory(
                source,
                command_rate_hz=settings.command_rate_hz,
                speed_scale=trial.speed_scale,
            )

    if prompt:
        print("THE ARM WILL MOVE: smooth approach, settle, then trajectory playback.")
        input("Press Enter to start, Ctrl-C to abort. ")

    with ForceTorqueClient.connect(settings.bridge) as ft, ForceTorqueConfigClient.connect(
        settings.bridge
    ) as cfg, SawyerRobotClient.connect(settings.bridge) as robot:
        original_settings = cfg.settings()
        original_comms = cfg.communications()
        original_configuration = cfg.configuration()
        identity = cfg.identity()

        state = robot.get_state()
        if not state.enabled:
            raise RuntimeError("Robot is not enabled")

        if session_dir is None:
            actual_session_dir = next_session_dir(output_root.expanduser())
        else:
            actual_session_dir = session_dir.expanduser()
            actual_session_dir.mkdir(parents=True, exist_ok=False)

        session_metadata = {
            "created_wall_s": time.time(),
            "bridge": settings.bridge,
            "clock_note": (
                "received_ns and command bridge timestamps use the bridge monotonic clock; "
                "they are software reception/publication times, not physical measurement "
                "or actuation times"
            ),
            "local_clock_note": (
                "local_* timestamps use this client process monotonic clock and must not be "
                "subtracted from bridge timestamps"
            ),
            "trajectory": {
                "path": str(source),
                "played_rate_hz": settings.command_rate_hz,
                "time_scaling": "q_scaled(t) = q_source(speed_scale * t)",
            },
            "trials_requested": [
                {
                    "rate": trial.sensor_rate_hz,
                    "filter": trial.filter_code,
                    "speed_scale": trial.speed_scale,
                    "name": trial.name,
                }
                for trial in trials
            ],
            "sensor_identity": proto_dict(identity),
            "sensor_configuration_at_start": proto_dict(original_configuration),
            "sensor_settings_at_start": proto_dict(original_settings),
            "sensor_communications_at_start": proto_dict(original_comms),
            "bias_note": "bias is never changed by run_trajectory_experiment",
        }
        write_json(actual_session_dir / "metadata.json", session_metadata)

        summaries: list[dict[str, Any]] = []
        try:
            for trial_index, trial in enumerate(trials, start=1):
                trajectory = cache[trial.speed_scale]
                comms, sensor_settings = _apply_sensor_trial(cfg, trial)
                label = _describe_effective(comms, sensor_settings)
                _wait_for_valid_ft(ft, settings=settings)

                approach = smooth_joint_move(
                    robot,
                    trajectory.positions[0],
                    settings=settings.approach,
                )
                time.sleep(settings.settle_s)

                ft_initial = _wait_for_valid_ft(ft, settings=settings)
                joint_initial = _latest_joint_cursor(robot)
                if ft_initial.history.clock_id != joint_initial.clock_id:
                    raise RuntimeError(
                        "F/T and joint histories do not share the same bridge clock_id"
                    )

                ft_capture = _ForceTorqueCapture(
                    settings.bridge,
                    ft_initial.history.session_id,
                    ft_initial.history.newest_id,
                    ft_initial.history.clock_id,
                    settings,
                )
                joint_capture = _JointCapture(
                    settings.bridge,
                    joint_initial.session_id,
                    joint_initial.sample_id,
                    joint_initial.clock_id,
                    settings,
                )
                commands: list[dict[str, Any]] = []
                play_error = None
                ft_capture.start()
                joint_capture.start()
                try:
                    time.sleep(settings.pre_roll_s)
                    _play(
                        robot,
                        trajectory.positions,
                        rate_hz=settings.command_rate_hz,
                        trial_index=trial_index,
                        commands=commands,
                        timeout_s=settings.command_timeout_s,
                    )
                    time.sleep(settings.post_roll_s)
                except (RuntimeError, grpc.RpcError) as exc:
                    play_error = str(exc)
                    try:
                        robot.stop()
                    except (BridgeError, grpc.RpcError):
                        pass
                finally:
                    ft_capture.stop()
                    joint_capture.stop()

                summary = _capture_summary(ft_capture, joint_capture, commands)
                requested = {
                    "rate": trial.sensor_rate_hz,
                    "filter": trial.filter_code,
                    "speed_scale": trial.speed_scale,
                }
                effective = {
                    "communications": proto_dict(comms),
                    "settings": proto_dict(sensor_settings),
                    "configuration": proto_dict(cfg.configuration()),
                }
                trial_metadata = {
                    "trial_index": trial_index,
                    "requested": requested,
                    "label": label,
                    "name": trial.name,
                    "trajectory": trajectory.metadata,
                    "effective": effective,
                    "approach": approach,
                    "settle_s": settings.settle_s,
                    "pre_roll_s": settings.pre_roll_s,
                    "post_roll_s": settings.post_roll_s,
                    "ft_start_history": proto_dict(ft_initial.history),
                    "joint_start_state": proto_dict(joint_initial),
                    "summary": summary,
                    "play_error": play_error,
                }
                speed_label = f"speed-{trial.speed_scale:.2f}"
                name = trial.name or speed_label
                trial_dir = actual_session_dir / (
                    f"trial-{trial_index:02d}_{slugify(label)}_{slugify(name)}"
                )
                _save_trial(trial_dir, trial_metadata, ft_capture, joint_capture, commands)
                if settings.plot:
                    _plot_trial(
                        ft_capture.samples,
                        title=f"{label} - {len(ft_capture.samples)} F/T samples",
                        path=trial_dir / "ft.png",
                        show=settings.show_plot,
                    )

                summaries.append(
                    {
                        "trial_index": trial_index,
                        "label": label,
                        "name": trial.name,
                        "speed_scale": trial.speed_scale,
                        "folder": trial_dir.name,
                        **summary,
                        "play_error": play_error,
                    }
                )
                write_json(actual_session_dir / "summary.json", summaries)

                if play_error:
                    raise RuntimeError(play_error)
                if ft_capture.fatal_error:
                    raise RuntimeError(ft_capture.fatal_error)
                if joint_capture.fatal_error:
                    raise RuntimeError(joint_capture.fatal_error)
                time.sleep(settings.between_s)
        finally:
            if restore_after:
                failures = _restore_config(cfg, original_settings, original_comms)
                if failures:
                    raise RuntimeError("Sensor setting restoration failed: " + "; ".join(failures))

    return actual_session_dir, summaries
