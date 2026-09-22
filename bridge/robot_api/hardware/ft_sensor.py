"""Single-owner ATI RDT acquisition, explicit bias, validated scaling and history."""

import math
import os
import socket
import struct
import threading
import time
from typing import cast

from .ft_acquisition import summarize
from .ft_buffer import SampleBuffer
from .ft_config import NetBoxConfig, NetBoxError

_RECORD = struct.Struct("!IIIiiiiii")
_COMMAND = struct.Struct("!HHI")


class FTSensorManager:
    def __init__(
        self,
        host=None,
        config=None,
        capacity=35000,
        timeout_s=1.0,
        settle_s=1.0,
        socket_factory=socket.socket,
    ):
        self.host = host or os.environ.get("FT_SENSOR_IP", "192.168.1.11")

        if self.host.lower() == "disabled":
            raise RuntimeError("FT sensor disabled via FT_SENSOR_IP=disabled")

        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("timeout_s must be positive")

        if not math.isfinite(settle_s) or settle_s < 0:
            raise ValueError("settle_s must be nonnegative")

        self.config = config or NetBoxConfig(self.host)
        self.buffer = SampleBuffer(capacity)
        self.timeout_s, self.settle_s = timeout_s, settle_s
        self._socket_factory = socket_factory
        self._stop = threading.Event()
        self._lifecycle = threading.RLock()
        self._thread = None
        self._version = 0
        self._error = "starting"
        self._zero_next = False
        self.start()

    @property
    def error(self):
        return self._error

    def start(self):

        with self._lifecycle:
            if self._thread is not None and self._thread.is_alive():
                return

            self._stop.clear()
            self._thread = threading.Thread(
                target=self._reader_loop, daemon=True, name="FTSensorReader"
            )
            self._thread.start()

    def close(self):

        with self._lifecycle:
            self._stop.set()

            if self._thread is not None:
                self._thread.join(timeout=8.0)
                if self._thread.is_alive():
                    raise RuntimeError("FT reader did not stop; configuration was not changed")
                self._thread = None

            self._error = "stopped"

    def change(self, action):
        """Stop/join before HTTP writes; restart in a new session even on failure."""

        with self._lifecycle:
            self.close()
            self.buffer.reset()

            try:
                return action(self.config)
            finally:
                self.start()

    def zero(self, timeout=5.0):
        """Explicit zero; success requires reception of a post-settling sample."""

        with self._lifecycle:
            self.close()
            self.buffer.reset()
            self._zero_next = True
            self.start()
            deadline = time.monotonic() + timeout

            while time.monotonic() < deadline:
                if self.buffer.latest() is not None and not self._error:
                    return True
                time.sleep(0.01)

            # No deferred zero is allowed after reporting failure.
            self.close()
            self._zero_next = False
            self.start()

            return False

    def query(self, max_age_s=0.1, **kwargs):
        error = "stopped" if self._stop.is_set() else self._error
        result = summarize(self.buffer.query(**kwargs), max_age_s, error)
        result["reader_error"] = error

        if result["session_id"] != self.buffer.session_id:
            result["valid"] = False
            result["mean"] = []

            reasons = cast(list[str], result["reasons"])
            reasons.append("session_changed_during_query")

        return result

    def latest(self):
        row = self.buffer.latest()

        if row is None or self._error or time.monotonic_ns() - row["received_ns"] > 100_000_000:
            return None

        return dict(
            zip(("fx", "fy", "fz", "tx", "ty", "tz"), row["values"]),
            seq=row["rdt_sequence"],
            status=row["status"],
            timestamp=row["wall_s"],
            **{
                k: row[k]
                for k in ("received_ns", "sample_id", "config_version", "force_unit", "torque_unit")
            },
        )

    def _scaling(self):
        # Strict read: never silently fall back to 1 or 1e6 for missing XML fields.
        raw = self.config.raw()
        try:
            cpf, cpt = int(raw["cfgcpf"]), int(raw["cfgcpt"])
            force_unit, torque_unit = raw["scfgfu"].strip(), raw["scfgtu"].strip()
            if cpf <= 0 or cpt <= 0 or not force_unit or not torque_unit:
                raise ValueError("invalid scaling")
        except (KeyError, TypeError, ValueError) as exc:
            raise NetBoxError("Missing/invalid active sensor scaling") from exc
        if str(raw.get("comrdte", "")).strip().lower() not in ("1", "enabled", "true"):
            raise NetBoxError("RDT is disabled; enable it explicitly with the config API")
        return cpf, cpt, force_unit, torque_unit

    @staticmethod
    def _send(sock, command):
        sock.send(_COMMAND.pack(0x1234, command, 0))

    def _reader_loop(self):

        while not self._stop.is_set():
            sock = None

            try:
                self._error = "connecting"

                if self._stop.wait(self.settle_s):
                    break

                cpf, cpt, fu, tu = self._scaling()

                if self._stop.is_set():
                    break

                sock = self._socket_factory(socket.AF_INET, socket.SOCK_DGRAM)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
                sock.settimeout(0.1)
                sock.connect((self.host, 49152))

                if self._zero_next:
                    self._send(sock, 0x0042)
                    self._zero_next = False

                self.buffer.reset()
                self._version += 1
                self._send(sock, 0x0002)
                previous = None
                ready_at = time.monotonic() + self.settle_s
                last_packet = time.monotonic()
                accepting = False

                while not self._stop.is_set():
                    try:
                        packet = sock.recv(4096)
                    except socket.timeout as exc:
                        if time.monotonic() - last_packet >= self.timeout_s:
                            raise TimeoutError("No RDT packets within timeout") from exc
                        continue

                    received_ns, wall_s = time.monotonic_ns(), time.time()

                    if len(packet) != _RECORD.size:
                        raise ValueError("Malformed RDT packet: expected 36 bytes")

                    rdt, ft, status, *counts = _RECORD.unpack(packet)
                    last_packet = time.monotonic()

                    if last_packet < ready_at:
                        previous = rdt
                        continue

                    delta = 1 if previous is None else (rdt - previous) & 0xFFFFFFFF

                    if delta == 0:  # duplicate
                        continue

                    if delta >= 0x80000000:
                        raise ValueError("RDT sequence regressed; restarting acquisition session")

                    previous = rdt

                    if not accepting:
                        self.buffer.reset()
                        accepting = True

                    values = [x / (cpf if i < 3 else cpt) for i, x in enumerate(counts)]

                    self.buffer.append(
                        {
                            "values": values,
                            "rdt_sequence": rdt,
                            "ft_sequence": ft,
                            "status": status,
                            "wall_s": wall_s,
                            "config_version": self._version,
                            "force_unit": fu,
                            "torque_unit": tu,
                            "dropped_before": delta - 1,
                        },
                        received_ns,
                    )
                    self._error = ""

            except (OSError, NetBoxError, ValueError, struct.error) as exc:
                self._error = str(exc)

            finally:
                if sock is not None:
                    try:
                        self._send(sock, 0x0000)
                    except OSError:
                        pass
                    try:
                        sock.close()
                    except OSError:
                        pass

            if not self._stop.is_set():
                self._stop.wait(0.5)
