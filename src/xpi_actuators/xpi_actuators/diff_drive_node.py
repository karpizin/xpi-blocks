"""Differential drive controller: cmd_vel (Twist) -> left and right motor commands.

Subscribes to ``cmd_vel`` (geometry_msgs/Twist; remap it, e.g. to the safe output of
cmd_vel_mux_node) and streams the wheel commands (std_msgs/Float32, -1.0..1.0) to the
motor driver topics at ``publish_rate``. The motor drivers' own watchdogs need that
stream; when ``cmd_vel`` goes silent for ``cmd_timeout`` the commands drop to 0.

Defaults fit tb6612_driver_node started with `ros2 run` (motor A = left, motor B = right).
"""
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32

from xpi_actuators.diff_drive import DiffDriveKinematics, ramp


class DiffDriveNode(Node):
    def __init__(self):
        super().__init__('diff_drive_node')
        self.declare_parameter('wheel_separation', 0.15)    # m, between the wheel centers
        self.declare_parameter('max_wheel_speed', 0.5)      # m/s of a wheel at full duty (1.0)
        self.declare_parameter('max_linear', 0.0)           # m/s limit, 0 = only max_wheel_speed
        self.declare_parameter('max_angular', 0.0)          # rad/s limit, 0 = none
        self.declare_parameter('max_accel', 0.0)            # duty per second when speeding up, 0 = off
        self.declare_parameter('left_topic', '/tb6612_driver_node/motor_a/cmd_vel')
        self.declare_parameter('right_topic', '/tb6612_driver_node/motor_b/cmd_vel')
        self.declare_parameter('invert_left', False)        # motor wired the other way round
        self.declare_parameter('invert_right', False)
        self.declare_parameter('cmd_timeout', 0.5)          # s without cmd_vel -> stop
        self.declare_parameter('publish_rate', 20.0)        # Hz

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.kinematics = DiffDriveKinematics(
            p('wheel_separation'), p('max_wheel_speed'), p('max_linear') or None, p('max_angular') or None)
        self.cmd_timeout = float(p('cmd_timeout'))
        rate = float(p('publish_rate'))
        if rate <= 0:
            raise ValueError('publish_rate must be > 0')
        self.max_step = max(0.0, float(p('max_accel'))) / rate
        self.signs = (-1.0 if p('invert_left') else 1.0, -1.0 if p('invert_right') else 1.0)

        self.target = (0.0, 0.0)
        self.output = (0.0, 0.0)
        self.last_cmd = None
        self.timed_out = True

        self.left_pub = self.create_publisher(Float32, p('left_topic'), 10)
        self.right_pub = self.create_publisher(Float32, p('right_topic'), 10)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        self.create_timer(1.0 / rate, self.timer_callback)
        self.get_logger().info(
            f"Diff drive: cmd_vel -> {p('left_topic')} / {p('right_topic')}, "
            f"separation {self.kinematics.wheel_separation} m, max wheel {self.kinematics.max_wheel_speed} m/s")

    def cmd_callback(self, msg: Twist):
        self.target = self.kinematics.wheel_commands(msg.linear.x, msg.angular.z)
        self.last_cmd = time.monotonic()
        self.timed_out = False

    def timer_callback(self):
        if self.last_cmd is None or (self.cmd_timeout > 0 and time.monotonic() - self.last_cmd > self.cmd_timeout):
            if not self.timed_out and self.last_cmd is not None:
                self.get_logger().warn(f'Diff drive: no cmd_vel for {self.cmd_timeout:.2f} s, stopping.')
            self.timed_out = True
            self.target = (0.0, 0.0)
        left = ramp(self.output[0], self.target[0], self.max_step)
        right = ramp(self.output[1], self.target[1], self.max_step)
        self.output = (left, right)
        self.left_pub.publish(Float32(data=float(left * self.signs[0])))
        self.right_pub.publish(Float32(data=float(right * self.signs[1])))

    def destroy_node(self):
        try:
            self.left_pub.publish(Float32(data=0.0))
            self.right_pub.publish(Float32(data=0.0))
        except Exception:  # noqa: BLE001 - shutting down anyway
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
