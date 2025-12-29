#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import math

class HeadingControllerNode(Node):
    """
    Controls the hexapod's heading (Yaw).
    Subscribes to IMU for current orientation and a target heading topic.
    Outputs Twist messages to rotate the robot.
    """
    def __init__(self):
        super().__init__('heading_controller_node')

        # 1. Parameters
        self.declare_parameter('kp', 1.5)      # Proportional gain
        self.declare_parameter('tolerance', 0.02) # Angle tolerance in radians (~1.1 deg)
        self.kp = self.get_parameter('kp').value
        self.tolerance = self.get_parameter('tolerance').value

        # 2. State
        self.current_yaw = 0.0
        self.target_yaw = None
        self.active = False

        # 3. Pubs & Subs
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.create_subscription(Float32, '/hexapod/target_heading', self.target_callback, 10)

        # 4. Control Timer (20 Hz)
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Heading Controller initialized. Waiting for target on /hexapod/target_heading (radians)')

    def imu_callback(self, msg):
        # Extract Yaw from Quaternion
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def target_callback(self, msg):
        self.target_yaw = msg.data
        self.active = True
        self.get_logger().info(f'New target heading set: {math.degrees(self.target_yaw):.1f} degrees')

    def control_loop(self):
        if not self.active or self.target_yaw is None:
            return

        # Calculate error
        error = self.target_yaw - self.current_yaw
        
        # Normalize error to [-pi, pi] to take the shortest path
        while error > math.pi: error -= 2 * math.pi
        while error < -math.pi: error += 2 * math.pi

        msg = Twist()
        if abs(error) > self.tolerance:
            # P-controller for angular velocity
            omega = error * self.kp
            # Clamp max speed
            msg.angular.z = max(min(omega, 1.0), -1.0)
        else:
            # Target reached
            msg.angular.z = 0.0
            self.active = False
            self.get_logger().info('Target heading reached.')

        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = HeadingControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
