#!/usr/bin/env python3
import sys
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

# Add path to scripts to import math
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from leg_kinematics import LegKinematics

class LegIKViewerNode(Node):
    def __init__(self):
        super().__init__('leg_ik_viewer_node')
        
        # 1. Geometry parameters (match URDF)
        self.declare_parameter('l1', 0.03) # Coxa
        self.declare_parameter('l2', 0.05) # Femur
        self.declare_parameter('l3', 0.08) # Tibia
        
        l1 = self.get_parameter('l1').value
        l2 = self.get_parameter('l2').value
        l3 = self.get_parameter('l3').value
        
        self.ik = LegKinematics(l1, l2, l3)
        
        # 2. Publishers & Subscribers
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_subscription(Point, '/leg/goal_point', self.point_callback, 10)
        
        # Initial state (leg in neutral position)
        self.current_point = Point(x=0.08, y=0.0, z=-0.04)
        self.publish_joints(self.current_point)
        
        self.get_logger().info('Leg IK Viewer Node started. Send goals to /leg/goal_point')

    def point_callback(self, msg):
        self.current_point = msg
        self.publish_joints(msg)

    def publish_joints(self, point):
        try:
            # Calculate IK
            # Note: our class formulas work correctly in mm or meters, 
            # as long as units are consistent. URDF is in meters.
            angles = self.ik.calculate_ik(point.x, point.y, point.z)
            
            # Prepare JointState message
            msg = JointState()
            msg.header = Header()
            msg.header.stamp = self.get_header_stamp()
            msg.name = ['coxa_joint', 'femur_joint', 'tibia_joint']
            msg.position = [angles[0], angles[1], angles[2]]
            
            self.joint_pub.publish(msg)
            
        except ValueError as e:
            self.get_logger().warn(f'IK Error: {e}')

    def get_header_stamp(self):
        return self.get_clock().now().to_msg()

def main(args=None):
    rclpy.init(args=args)
    node = LegIKViewerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
