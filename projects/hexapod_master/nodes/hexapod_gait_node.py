#!/usr/bin/env python3
import sys
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
import time

# Добавляем пути
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from gait_engine import GaitEngine

class HexapodGaitNode(Node):
    def __init__(self):
        super().__init__('hexapod_gait_node')
        
        # Parameters
        self.declare_parameter('gait_type', 'tripod')
        self.declare_parameter('step_height', 0.03)
        self.declare_parameter('step_length', 0.05)
        
        gait_type = self.get_parameter('gait_type').value
        step_h = self.get_parameter('step_height').value
        step_l = self.get_parameter('step_length').value
        
        self.gait = GaitEngine(step_height=step_h, step_length=step_l, gait_type=gait_type)
        
        self.current_vel = [0.0, 0.0]
        self.current_omega = 0.0
        
        # Publishers
        self.leg_pubs = {}
        leg_names = ['rf', 'rm', 'rb', 'lf', 'lm', 'lb']
        for name in leg_names:
            topic = f'/hexapod/{name}/gait_offset'
            self.leg_pubs[name] = self.create_publisher(Point, topic, 10)
            
        # Subscriber
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        
        # Timer (50 Hz)
        self.last_time = time.time()
        self.create_timer(0.02, self.update_gait)
        
        # Parameter Callback
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        self.get_logger().info(f'Hexapod Gait Node started. Gait: {gait_type.upper()}')

    def parameters_callback(self, params):
        for param in params:
            if param.name == 'gait_type':
                self.gait.set_gait_type(param.value)
                self.get_logger().info(f'Gait switched to: {param.value.upper()}')
            elif param.name == 'step_height':
                self.gait.step_height = param.value
            elif param.name == 'step_length':
                self.gait.step_length = param.value
        return rclpy.node.SetParametersResult(successful=True)

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