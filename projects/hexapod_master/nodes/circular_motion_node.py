#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import math

class CircularMotionNode(Node):
    """
    Commands the hexapod to move in a circle of a specific radius.
    Subscribes to:
      - /hexapod/set_circle_radius (Float32): Radius in meters (positive=left, negative=right)
      - /hexapod/set_circle_speed (Float32): Linear speed in m/s
    """
    def __init__(self):
        super().__init__('circular_motion_node')

        # 1. State
        self.radius = 0.0  # 0.0 means stop/straight
        self.speed = 0.0
        self.active = False

        # 2. Pubs & Subs
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Float32, '/hexapod/set_circle_radius', self.radius_callback, 10)
        self.create_subscription(Float32, '/hexapod/set_circle_speed', self.speed_callback, 10)

        # 3. Control Timer (10 Hz)
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Circular Motion Node initialized. Use /hexapod/set_circle_radius and speed.')

    def radius_callback(self, msg):
        self.radius = msg.data
        self.active = (abs(self.speed) > 0.001)
        self.get_logger().info(f'Radius set to: {self.radius:.2f}m')

    def speed_callback(self, msg):
        self.speed = msg.data
        self.active = (abs(self.speed) > 0.001)
        if not self.active:
            self.stop_robot()
        self.get_logger().info(f'Speed set to: {self.speed:.2f}m/s')

    def stop_robot(self):
        msg = Twist()
        self.cmd_pub.publish(msg)

    def control_loop(self):
        if not self.active:
            return

        msg = Twist()
        msg.linear.x = self.speed

        if abs(self.radius) > 0.05: # Prevent division by zero or tiny radius
            # omega = v / R
            msg.angular.z = self.speed / self.radius
        else:
            msg.angular.z = 0.0

        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CircularMotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
