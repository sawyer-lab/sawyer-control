"""ATI Net F/T reader owned exclusively by the bridge process."""

import os
import socket
import struct
import threading
import time

import rospy

_RDT_PORT = 49152
_RDT_HEADER = 0x1234
_SAMPLE_COUNT_INF = 0
_RESPONSE_FMT = "!IIIiiiiii"  # rdt_seq ft_seq status Fx Fy Fz Tx Ty Tz
_RESPONSE_SIZE = struct.calcsize(_RESPONSE_FMT)  # 36 bytes

_CMD_STOP = 0x0000
_CMD_START_REALTIME = 0x0002
_CMD_RESET_LATCH = 0x0041
_CMD_SET_BIAS = 0x0042


class FTSensorManager:
    """Owns a streaming ATI Net F/T connection. A background thread keeps the
    freshest sample cached; consumers call `latest()` (non-blocking) and
    `zero()` (re-bias, performed on the reader thread)."""

    def __init__(self, counts_per_force: int = 1_000_000, counts_per_torque: int = 1_000_000):
        ip = os.environ.get("FT_SENSOR_IP", "192.168.1.11")
        if ip.lower() == "disabled":
            raise RuntimeError("FT sensor disabled via FT_SENSOR_IP=disabled")
        self._cpf = counts_per_force
        self._cpt = counts_per_torque
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(1.0)
        self._sock.connect((ip, _RDT_PORT))
        self._send(_CMD_RESET_LATCH)
        self._send(_CMD_SET_BIAS)
        self._send(_CMD_START_REALTIME)

        self._latest = None              # freshest snapshot dict, or None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._bias_req = threading.Event()
        self._bias_done = threading.Event()
        self._thread = threading.Thread(target=self._reader_loop, daemon=True,
                                        name="FTSensorReader")
        self._thread.start()
        rospy.loginfo(f"FT sensor streaming from {ip}")

    def latest(self):
        """Freshest cached sample as a dict, or None if nothing read yet.

        Keys: fx fy fz tx ty tz (N / Nm), seq (rdt sequence), status,
        timestamp (time.time() when cached)."""
        with self._lock:
            return dict(self._latest) if self._latest is not None else None

    def zero(self, timeout: float = 1.0) -> bool:
        """Re-bias at the current load. The bias command + buffer flush happen
        on the reader thread (sole socket owner); this blocks until done so a
        caller can rely on the next `latest()` being post-bias."""
        self._bias_done.clear()
        self._bias_req.set()
        return self._bias_done.wait(timeout)

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
        try:
            self._send(_CMD_STOP)
        finally:
            self._sock.close()

    # ── reader thread (the only code that touches the socket) ────────────────

    def _reader_loop(self) -> None:
        while not self._stop.is_set():
            if self._bias_req.is_set():
                self._send(_CMD_SET_BIAS)
                time.sleep(0.05)   # let the bias apply to the live stream
                self._drain()      # discard pre-bias packets in the buffer
                self._bias_req.clear()
                self._bias_done.set()

            raw = self._drain()
            if raw is None:
                try:
                    raw = self._sock.recv(_RESPONSE_SIZE)
                except (socket.timeout, OSError):
                    continue
            rdt, _ft, status, fx, fy, fz, tx, ty, tz = struct.unpack(_RESPONSE_FMT, raw)
            snap = {
                "fx": fx / self._cpf, "fy": fy / self._cpf, "fz": fz / self._cpf,
                "tx": tx / self._cpt, "ty": ty / self._cpt, "tz": tz / self._cpt,
                "seq": rdt, "status": status, "timestamp": time.time(),
            }
            with self._lock:
                self._latest = snap

    def _drain(self):
        """Read all currently-queued packets non-blocking; return the last."""
        last = None
        self._sock.setblocking(False)
        try:
            while True:
                try:
                    last = self._sock.recv(_RESPONSE_SIZE)
                except (BlockingIOError, OSError):
                    break
        finally:
            self._sock.setblocking(True)
            self._sock.settimeout(1.0)
        return last

    def _send(self, command: int) -> None:
        self._sock.send(struct.pack("!HHI", _RDT_HEADER, command, _SAMPLE_COUNT_INF))
