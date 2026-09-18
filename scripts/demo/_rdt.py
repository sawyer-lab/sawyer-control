"""Minimal RDT reader for the demo scripts.

Standalone on purpose: the bridge's driver (`ft_sensor.py`) imports rospy and
only runs inside the container, and it keeps a freshest-sample cache rather than
every sample. These tools need a lossless capture from the host, so they talk to
the sensor directly.

Not for production use - the bridge owns the sensor at runtime.
"""

from __future__ import annotations

import socket
import struct
import time

RDT_PORT = 49152
RDT_HEADER = 0x1234
RECORD = struct.Struct("!IIIiiiiii")      # rdt_seq ft_seq status Fx Fy Fz Tx Ty Tz
CMD_STOP, CMD_START_REALTIME = 0x0000, 0x0002
AXES = ("Fx", "Fy", "Fz", "Tx", "Ty", "Tz")


def stream(host: str, seconds: float, scale_force: float = 1.0, scale_torque: float = 1.0,
           timeout: float = 2.0):
    """Yield (elapsed_s, row) for `seconds`, then a final stats dict.

    The generator form exists so a caller can do work between samples - the
    guided step-response test prints prompts on a schedule while the capture
    keeps running - without a second thread. `capture` below is the batch
    wrapper for callers that just want everything.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(timeout)
    sock.connect((host, RDT_PORT))
    sock.send(struct.pack("!HHI", RDT_HEADER, CMD_START_REALTIME, 0))

    count, dropped, previous = 0, 0, None
    started = time.time()
    deadline = started + seconds
    try:
        while True:
            now = time.time()
            if now >= deadline:
                break
            try:
                packet = sock.recv(RECORD.size)
            except socket.timeout:
                break
            sequence, _ft, status, *counts = RECORD.unpack(packet)
            if previous is not None and sequence != previous + 1:
                dropped += sequence - previous - 1
            previous = sequence
            count += 1
            yield now - started, [counts[0] / scale_force, counts[1] / scale_force,
                                  counts[2] / scale_force, counts[3] / scale_torque,
                                  counts[4] / scale_torque, counts[5] / scale_torque, status]
    finally:
        try:
            sock.send(struct.pack("!HHI", RDT_HEADER, CMD_STOP, 0))
        finally:
            sock.close()

    elapsed = time.time() - started
    return {"samples": count, "elapsed_s": elapsed,
            "measured_hz": count / elapsed if elapsed else 0.0, "dropped": dropped}


def capture(host: str, seconds: float, scale_force: float = 1.0, scale_torque: float = 1.0,
            timeout: float = 2.0):
    """Stream for `seconds`, returning (rows, stats).

    `rows` is a list of [fx, fy, fz, tx, ty, tz, status] in user units. `stats`
    reports sample count, measured rate, and packets dropped (from gaps in the
    RDT sequence number, which the device increments per packet).
    """
    rows = []
    generator = stream(host, seconds, scale_force, scale_torque, timeout)
    while True:
        try:
            _elapsed, row = next(generator)
        except StopIteration as done:
            return rows, done.value
        rows.append(row)
