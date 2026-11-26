"""Shared safety helpers for motor driver nodes.

Two things every speed-controlled motor driver needs:

* ``sanitize_speed`` - turn an incoming command into a finite value in
  [-1.0, 1.0]. ``NaN``/``inf`` become 0.0 (stop). Note that a plain
  ``max(-1, min(1, x))`` maps ``NaN`` to full speed, because every comparison
  with ``NaN`` is False.

* ``CommandWatchdog`` - a dead-man switch. Each channel must keep receiving
  commands; if a channel has been non-zero and nothing arrived for
  ``timeout`` seconds, the watchdog reports it so the node can stop that
  motor. Publishers must therefore stream commands (e.g. 10 Hz), not send a
  single message. ``timeout <= 0`` disables the watchdog.

This module has no ROS dependency so it can be unit-tested directly.
"""
import math
import threading
import time

DEFAULT_CMD_TIMEOUT_S = 0.5


def sanitize_speed(value, limit=1.0):
    """Return ``value`` as a float clamped to [-limit, limit]; non-finite -> 0.0."""
    try:
        speed = float(value)
    except (TypeError, ValueError):
        return 0.0
    if not math.isfinite(speed):
        return 0.0
    limit = abs(float(limit))
    if speed > limit:
        return limit
    if speed < -limit:
        return -limit
    return speed


class CommandWatchdog:
    """Tracks the last command time per channel and reports stale channels."""

    def __init__(self, timeout, clock=time.monotonic):
        self.timeout = float(timeout)
        self._clock = clock
        self._lock = threading.Lock()
        self._last = {}      # channel -> time of last command
        self._active = set()  # channels currently driven with a non-zero command

    @property
    def enabled(self):
        return self.timeout > 0

    def feed(self, channel, speed):
        """Record a command for ``channel``. ``speed`` is the sanitized value applied."""
        with self._lock:
            self._last[channel] = self._clock()
            if speed != 0.0:
                self._active.add(channel)
            else:
                self._active.discard(channel)

    def expired(self):
        """Return channels that are moving but have not been fed within ``timeout``.

        Returned channels are marked as stopped, so each expiry is reported once.
        """
        if not self.enabled:
            return []
        now = self._clock()
        with self._lock:
            stale = [ch for ch in self._active if now - self._last.get(ch, now) > self.timeout]
            for ch in stale:
                self._active.discard(ch)
            return stale

    def check_period(self):
        """Suggested timer period for calling ``expired()`` (a fifth of the timeout, 20-100 ms)."""
        if not self.enabled:
            return 0.1
        return min(0.1, max(0.02, self.timeout / 5.0))
