"""Unit tests for xpi_actuators.motor_safety (no ROS required)."""
import math

import pytest
from xpi_actuators.motor_safety import CommandWatchdog, sanitize_speed


class FakeClock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


@pytest.mark.parametrize('value, expected', [
    (0.5, 0.5), (-0.25, -0.25), (0.0, 0.0),
    (2.0, 1.0), (-7.0, -1.0), (1e9, 1.0),
    (math.nan, 0.0), (math.inf, 0.0), (-math.inf, 0.0),
    (None, 0.0), ('fast', 0.0), ('0.3', 0.3),
])
def test_sanitize_speed(value, expected):
    assert sanitize_speed(value) == expected


def test_naive_clamp_maps_nan_to_full_speed():
    # Documents why sanitize_speed exists: the old clamp sent NaN to full speed.
    assert max(-1.0, min(1.0, math.nan)) == 1.0
    assert sanitize_speed(math.nan) == 0.0


def test_sanitize_speed_respects_limit():
    assert sanitize_speed(0.9, limit=0.6) == 0.6
    assert sanitize_speed(-0.9, limit=0.6) == -0.6
    assert sanitize_speed(0.9, limit=-0.6) == 0.6  # sign of the limit is ignored


def test_watchdog_expires_moving_channel_once():
    clock = FakeClock()
    wd = CommandWatchdog(0.5, clock=clock)
    wd.feed('motor_a', 0.8)
    clock.advance(0.4)
    assert wd.expired() == []
    clock.advance(0.2)
    assert wd.expired() == ['motor_a']
    assert wd.expired() == []  # reported once, until the next non-zero command


def test_watchdog_keepalive_prevents_expiry():
    clock = FakeClock()
    wd = CommandWatchdog(0.5, clock=clock)
    for _ in range(20):
        wd.feed('motor_a', 0.8)
        clock.advance(0.1)
        assert wd.expired() == []


def test_watchdog_ignores_stopped_channel():
    clock = FakeClock()
    wd = CommandWatchdog(0.5, clock=clock)
    wd.feed('motor_a', 0.8)
    wd.feed('motor_a', 0.0)
    clock.advance(10)
    assert wd.expired() == []


def test_watchdog_channels_are_independent():
    clock = FakeClock()
    wd = CommandWatchdog(0.5, clock=clock)
    wd.feed('motor_a', 0.5)
    wd.feed('motor_b', -0.5)
    clock.advance(0.3)
    wd.feed('motor_b', -0.5)
    clock.advance(0.3)
    assert wd.expired() == ['motor_a']


def test_watchdog_disabled():
    clock = FakeClock()
    wd = CommandWatchdog(0.0, clock=clock)
    assert not wd.enabled
    wd.feed('motor_a', 1.0)
    clock.advance(100)
    assert wd.expired() == []


def test_check_period_bounds():
    assert CommandWatchdog(0.5).check_period() == 0.1
    assert CommandWatchdog(0.05).check_period() == 0.02
    assert CommandWatchdog(0.2).check_period() == pytest.approx(0.04)
