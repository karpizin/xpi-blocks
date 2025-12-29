#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose
import math

class AutoLevelerNode(Node):
    def __init__(self):
        super().__init__('auto_leveler_node')
        
        # 1. Parameters
        self.declare_parameter('kp', 0.5) # Gain coefficient
        self.kp = self.get_parameter('kp').value
        
        # 2. Publishers & Subscribers
        self.pose_pub = self.create_publisher(Pose, '/hexapod/body_pose', 10)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        
        # State
        self.current_pose = Pose()
        self.current_pose.position.z = -0.08 # Initial height
        
        self.get_logger().info('Auto-Leveler Node initialized. Watching /imu/data')

    def imu_callback(self, msg):
        # In a real system, we would use a Kalman or Madgwick filter.
        # But for simplicity, we'll take angles from the accelerometer directly.
        accel = msg.linear_acceleration
        
        # Calculate Roll and Pitch from gravity acceleration
        roll_accel = math.atan2(accel.y, accel.z)
        pitch_accel = math.atan2(-accel.x, math.sqrt(accel.y**2 + accel.z**2))
        
        # Invert them to compensate for tilt
        target_roll = -roll_accel * self.kp
        target_pitch = -pitch_accel * self.kp
        
        # Publish new desired body pose
        # Note: For simplification, we pass angles through position or orientation.
        # In our body_node we currently only take position, let's update it.
        
        new_msg = Pose()
        new_msg.position.x = 0.0
        new_msg.position.y = 0.0
        new_msg.position.z = self.current_pose.position.z
        
        # Use orientation (quaternion)
        new_msg.orientation = self.euler_to_quaternion(target_roll, target_pitch, 0.0)
        
        self.pose_pub.publish(new_msg)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

from geometry_msgs.msg import Quaternion

def main(args=None):
    rclpy.init(args=args)
    node = AutoLevelerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
