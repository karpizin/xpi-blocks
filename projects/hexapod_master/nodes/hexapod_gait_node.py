#!/usr/bin/env python3
import sys
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
import time

# Add paths
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from gait_engine import GaitEngine

class HexapodGaitNode(Node):
    def __init__(self):
        super().__init__('hexapod_gait_node')
        
        self.gait = GaitEngine()
        self.current_vel = [0.0, 0.0]
        self.current_omega = 0.0
        
        # Publishers for each leg (offsets will be summed in body_node or go separately)
        # For simplicity, we publish to offset topics
        self.leg_pubs = {}
        leg_names = ['rf', 'rm', 'rb', 'lf', 'lm', 'lb']
        for name in leg_names:
            topic = f'/hexapod/{name}/gait_offset'
            self.leg_pubs[name] = self.create_publisher(Point, topic, 10)
            
        # Subscriber for movement commands
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        
        # Gait update timer (50 Hz)
        self.last_time = time.time()
        self.create_timer(0.02, self.update_gait)
        
        self.get_logger().info('Hexapod Gait Node (Tripod) started. Waiting for /cmd_vel')

    def cmd_callback(self, msg):
        self.current_vel = [msg.linear.x, msg.linear.y]
        self.current_omega = msg.angular.z

    def update_gait(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        
        offsets = self.gait.calculate_offsets(self.current_vel, self.current_omega, dt)
        
        for name, off in offsets.items():
            msg = Point()
            msg.x = off['x']
            msg.y = off['y']
            msg.z = off['z']
            self.leg_pubs[name].publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = HexapodGaitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
