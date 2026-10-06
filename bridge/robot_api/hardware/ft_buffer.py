"""Bounded, non-destructive histories in the bridge's monotonic clock domain."""

import threading
import time
import uuid
from collections import deque
from copy import deepcopy

CLOCK_ID = uuid.uuid4().hex  # changes on bridge process restart


class SampleBuffer:
    def __init__(self, capacity=35000):
        if capacity < 1:
            raise ValueError("capacity must be positive")

        self._rows = deque(maxlen=capacity)
        self._lock = threading.Lock()
        self._next = 1
        self._evicted_through_ns = 0
        self._session = uuid.uuid4().hex
        self._coverage_start_ns = time.monotonic_ns()

    @property
    def session_id(self):
        with self._lock:
            return self._session

    def reset(self):
        with self._lock:
            self._rows.clear()
            self._next = 1
            self._evicted_through_ns = 0
            self._session = uuid.uuid4().hex
            self._coverage_start_ns = time.monotonic_ns()

    def append(self, payload, received_ns=None):
        stamp = time.monotonic_ns() if received_ns is None else received_ns
        with self._lock:
            if self._rows and stamp < self._rows[-1]["received_ns"]:
                raise ValueError("timestamps must be monotonic")

            if len(self._rows) == self._rows.maxlen:
                self._evicted_through_ns = self._rows[0]["received_ns"]

            row = dict(deepcopy(payload), sample_id=self._next, received_ns=stamp)
            self._next += 1
            self._rows.append(row)

            return deepcopy(row)

    def query(self, after_id=0, session_id="", start_ns=0, end_ns=0, limit=1000):
        """Cursor: id > after_id. Time window: start <= t < end.

        Empty session is allowed only to start a query. A cursor requires its
        session so a reconnect cannot silently turn into an empty response.
        The query never consumes data. Limits paginate; no implicit dropping.
        """
        if not 1 <= limit <= 35000:
            raise ValueError("limit must be 1..35000")

        if min(after_id, start_ns, end_ns) < 0:
            raise ValueError("negative cursor/time")

        if after_id and not session_id:
            raise ValueError("after_id requires session_id")

        if after_id and start_ns:
            raise ValueError("choose cursor or time window")

        now = time.monotonic_ns()
        end = end_ns or now

        if end > now or start_ns > end:
            raise ValueError("window must end at or before bridge now")
        with self._lock:
            mismatch = bool(session_id and session_id != self._session)
            oldest = self._rows[0]["sample_id"] if self._rows else self._next
            newest = self._rows[-1]["sample_id"] if self._rows else self._next - 1
            overflow = (
                after_id < oldest - 1
                if not start_ns
                else bool(self._evicted_through_ns and start_ns <= self._evicted_through_ns)
            )
            snapshot = list(self._rows)
            current_session = self._session
            incomplete = bool(start_ns and start_ns < self._coverage_start_ns)

        # Only snapshot references under the lock. Rows are immutable internally;
        # filtering/copying large histories must not block the UDP writer.
        candidates = (
            []
            if mismatch
            else [
                r
                for r in snapshot
                if r["sample_id"] > after_id and start_ns <= r["received_ns"] < end
            ]
        )
        rows = deepcopy(candidates[:limit])

        return {
            "rows": rows,
            "session_id": current_session,
            "clock_id": CLOCK_ID,
            "server_ns": now,
            "start_ns": start_ns,
            "end_ns": end,
            "oldest_id": oldest,
            "newest_id": newest,
            "next_id": rows[-1]["sample_id"] if rows else (0 if mismatch else after_id),
            "overflow": overflow,
            "session_changed": mismatch,
            "cursor_ahead": bool(not mismatch and after_id > newest),
            "truncated": len(candidates) > limit,
            "incomplete_window": incomplete,
        }

    def latest(self):
        with self._lock:
            return deepcopy(self._rows[-1]) if self._rows else None
