"""Differential drive: kinematics, limits, timeout and the published motor stream."""
import math
import threading

import pytest
from geometry_msgs.msg import Twist

from xpi_actuators.diff_drive import DiffDriveKinematics, ramp


def twist(v=0.0, w=0.0):
    t = Twist()
    t.linear.x, t.angular.z = v, w
    return t


# ---------------------------------------------------------------- kinematics
@pytest.mark.parametrize('v,w,expected', [
    (0.25, 0.0, (0.5, 0.5)),        # straight, half of max_wheel_speed 0.5
    (-0.25, 0.0, (-0.5, -0.5)),
    (0.0, 2.0, (-0.3, 0.3)),        # spin left: 2 rad/s * 0.075 m = 0.15 m/s per wheel
    (0.2, 1.0, (0.25, 0.55)),       # arc to the left: right wheel faster
])
def test_wheel_commands(v, w, expected):
    k = DiffDriveKinematics(wheel_separation=0.15, max_wheel_speed=0.5)
    assert k.wheel_commands(v, w) == pytest.approx(expected)


def test_saturation_keeps_curvature():
    k = DiffDriveKinematics(0.15, 0.5)
    left, right = k.wheel_commands(1.0, 4.0)       # wants 0.7 and 1.3 m/s
    assert max(abs(left), abs(right)) == pytest.approx(1.0)
    assert left / right == pytest.approx(0.7 / 1.3)


@pytest.mark.parametrize('v,w,same_as', [((math.nan), 0.0, (0.0, 0.0)), (0.2, math.inf, (0.2, 0.0)),
                                           ('fast', None, (0.0, 0.0))])
def test_non_finite_counts_as_zero(v, w, same_as):
    k = DiffDriveKinematics(0.15, 0.5)
    assert k.wheel_commands(v, w) == k.wheel_commands(*same_as)


def test_linear_and_angular_limits():
    k = DiffDriveKinematics(0.15, 0.5, max_linear=0.1, max_angular=1.0)
    assert k.wheel_commands(5.0, 0.0) == pytest.approx((0.2, 0.2))
    assert k.wheel_commands(0.0, 10.0) == pytest.approx((-0.15, 0.15))


def test_bad_geometry():
    with pytest.raises(ValueError):
        DiffDriveKinematics(0.0, 0.5)


def test_ramp_limits_only_speeding_up():
    assert ramp(0.0, 1.0, 0.1) == pytest.approx(0.1)
    assert ramp(0.9, 1.0, 0.5) == pytest.approx(1.0)
    assert ramp(0.8, 0.0, 0.1) == 0.0           # stop at once
    assert ramp(0.8, 0.3, 0.1) == 0.3           # slow down at once
    assert ramp(0.5, -0.5, 0.1) == 0.0          # reverse: stop first
    assert ramp(0.0, -0.5, 0.1) == pytest.approx(-0.1)
    assert ramp(0.0, 1.0, 0.0) == 1.0           # disabled


# ---------------------------------------------------------------- node
@pytest.fixture
def diff(ros):
    from xpi_actuators.diff_drive_node import DiffDriveNode

    def make(**params):
        ros.OVERRIDES.update(params)
        node = DiffDriveNode()
        left = next(p for p in node.publishers_ if p.topic == node.get_parameter('left_topic').value)
        right = next(p for p in node.publishers_ if p.topic == node.get_parameter('right_topic').value)
        cmd = next(s for s in node.subscriptions_ if s.topic == 'cmd_vel').callback
        return node, cmd, left, right
    return make


def last(pub):
    return pub.messages[-1].data


def test_default_topics_are_the_tb6612_driver():
    from xpi_actuators.diff_drive_node import DiffDriveNode  # noqa: F401
    import xpi_actuators.tb6612_driver_node as tb
    src = open(tb.__file__).read()
    assert "super().__init__('tb6612_driver_node')" in src
    assert "'~/motor_a/cmd_vel'" in src and "'~/motor_b/cmd_vel'" in src


def test_streams_wheel_commands(diff):
    node, cmd, left, right = diff()
    cmd(twist(0.25, 0.0))
    node.timer_callback()
    node.timer_callback()
    assert (last(left), last(right)) == pytest.approx((0.5, 0.5))
    assert len(left.messages) == 2              # streamed every tick, for the motor watchdogs


def test_stops_when_cmd_vel_goes_silent(diff):
    node, cmd, left, right = diff(cmd_timeout=0.05)
    cmd(twist(0.25, 0.0))
    node.timer_callback()
    assert last(left) == pytest.approx(0.5)
    threading.Event().wait(0.08)
    node.timer_callback()
    assert (last(left), last(right)) == (0.0, 0.0)
    from ros_stub import rclpy_stub
    assert any('no cmd_vel' in r['msg'] for r in rclpy_stub.LOG)


def test_zero_before_first_command(diff):
    node, cmd, left, right = diff()
    node.timer_callback()
    assert (last(left), last(right)) == (0.0, 0.0)


def test_invert_and_custom_topics(diff):
    node, cmd, left, right = diff(left_topic='/l298_driver_node/motor_a/cmd_speed',
                                  right_topic='/l298_driver_node/motor_b/cmd_speed', invert_right=True)
    cmd(twist(0.25, 0.0))
    node.timer_callback()
    assert (left.topic, last(left), last(right)) == ('/l298_driver_node/motor_a/cmd_speed', 0.5, -0.5)


def test_acceleration_limit(diff):
    node, cmd, left, right = diff(max_accel=2.0, publish_rate=20.0)   # 0.1 per tick
    cmd(twist(0.5, 0.0))
    for _ in range(3):
        node.timer_callback()
    assert last(left) == pytest.approx(0.3)
    cmd(twist(0.0, 0.0))
    node.timer_callback()
    assert last(left) == 0.0                     # stop is immediate


def test_destroy_sends_stop(diff):
    node, cmd, left, right = diff()
    cmd(twist(0.25, 0.0))
    node.timer_callback()
    node.destroy_node()
    assert (last(left), last(right)) == (0.0, 0.0)
