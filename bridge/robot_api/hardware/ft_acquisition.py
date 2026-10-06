"""Pure interval reduction. No socket, ROS or independent acquisition thread."""

import math

from . import ft_status


def summarize(batch, max_age_s=0.1, reader_error=""):

    if not math.isfinite(max_age_s) or max_age_s <= 0:
        raise ValueError("max_age_s must be finite and positive")

    rows = batch["rows"]
    reasons = []

    for key in ("overflow", "session_changed", "cursor_ahead", "truncated", "incomplete_window"):
        if batch[key]:
            reasons.append(key)

    if reader_error:
        reasons.append("reader_error")

    result = dict(
        batch,
        valid=False,
        reasons=reasons,
        count=len(rows),
        mean=[],
        age_s=0.0,
        first_ns=0,
        last_ns=0,
        status=0,
        dropped=sum(r["dropped_before"] for r in rows),
        config_version=0,
        force_unit="",
        torque_unit="",
    )

    if not rows:
        reasons.append("no_samples")

        return result

    # Age of the newest sample at the buffer query time.
    age_s = max(
        0.0,
        (batch["server_ns"] - rows[-1]["received_ns"]) / 1e9,
    )

    result.update(
        first_ns=rows[0]["received_ns"],
        last_ns=rows[-1]["received_ns"],
        age_s=age_s,
        config_version=rows[-1]["config_version"],
        force_unit=rows[-1]["force_unit"],
        torque_unit=rows[-1]["torque_unit"],
    )

    if age_s > max_age_s:
        reasons.append("stale")

    if result["dropped"]:
        reasons.append("packet_loss")
    versions = {(r["config_version"], r["force_unit"], r["torque_unit"]) for r in rows}

    if len(versions) != 1:
        reasons.append("mixed_configuration")

    if any(not ft_status.is_healthy(r["status"]) for r in rows):
        reasons.append("sensor_fault")

    for row in rows:
        result["status"] |= row["status"]

    # Never turn invalid/empty data into a zero force or a plausible mean.
    if not reasons:
        result["mean"] = [
            math.fsum(r["values"][axis] for r in rows) / len(rows) for axis in range(6)
        ]
        result["valid"] = True

    return result
