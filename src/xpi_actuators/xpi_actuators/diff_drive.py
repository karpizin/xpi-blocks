"""Differential drive: body velocity (Twist) -> left/right wheel commands.

No ROS dependency, so it can be unit-tested directly.

Wheel speeds in m/s are converted to the normalized commands the motor drivers take
(-1.0..1.0) by dividing by ``max_wheel_speed``, the wheel speed at full duty cycle.
When a wheel would exceed 1.0, both wheels are scaled by the same factor: the robot
keeps its curvature (turns on the same arc) instead of turning tighter than asked.
"""
import math


def _finite(value):
    try:
        v = float(value)
    except (TypeError, ValueError):
        return 0.0
    return v if math.isfinite(v) else 0.0


class DiffDriveKinematics:
    def __init__(self, wheel_separation, max_wheel_speed, max_linear=None, max_angular=None):
        if not (wheel_separation > 0 and max_wheel_speed > 0):
            raise ValueError('wheel_separation and max_wheel_speed must be > 0')
        self.wheel_separation = float(wheel_separation)
        self.max_wheel_speed = float(max_wheel_speed)
        self.max_linear = abs(max_linear) if max_linear else None
        self.max_angular = abs(max_angular) if max_angular else None

    def wheel_commands(self, linear_x, angular_z):
        """(left, right) in -1.0..1.0. Non-finite inputs count as 0 (stop)."""
        v, w = _finite(linear_x), _finite(angular_z)
        if self.max_linear is not None:
            v = max(-self.max_linear, min(self.max_linear, v))
        if self.max_angular is not None:
            w = max(-self.max_angular, min(self.max_angular, w))
        half = self.wheel_separation / 2.0
        left = (v - w * half) / self.max_wheel_speed
        right = (v + w * half) / self.max_wheel_speed
        peak = max(abs(left), abs(right))
        if peak > 1.0:
            left, right = left / peak, right / peak
        return left, right


def ramp(current, target, max_step):
    """Move ``current`` towards ``target`` by at most ``max_step`` when speeding up.

    Slowing down and stopping are never delayed: a stop must act at once.
    ``max_step <= 0`` disables the ramp.
    """
    if max_step <= 0:
        return target
    if target * current < 0:
        return 0.0   # reversing: stop first, the next steps ramp the other way
    if abs(target) > abs(current):
        return current + max(-max_step, min(max_step, target - current))
    return target
