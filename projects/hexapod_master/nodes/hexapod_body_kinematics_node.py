#!/usr/bin/env python3
import sys
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Quaternion
import yaml
import math

# Add paths
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from body_kinematics import BodyKinematics
from interpolator import Interpolator

class HexapodBodyNode(Node):
    def __init__(self):
        super().__init__('hexapod_body_node')
        
        # 1. Config Loading
        config_path = '/Users/slava/Documents/xpi-blocks/projects/hexapod_master/config/hexapod_config.yaml'
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        
        self.body_ik = BodyKinematics(self.config['legs'])
        
        # 2. Interpolator for pose (x, y, z, roll, pitch, yaw)
        # Initial height is taken from config
        h = self.config.get('default_height', 0.08)
        self.pose_interp = Interpolator([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], speed=0.05) # 5cm or 0.05rad per sec
        
        # 3. Publishers for each leg
        self.leg_pubs = {}
        self.gait_offsets = {}
        for leg_name in self.config['legs'].keys():
            topic = f'/hexapod/{leg_name}/goal_point'
            self.leg_pubs[leg_name] = self.create_publisher(Point, topic, 10)
            
            # Initialize gait offsets with zeros
            self.gait_offsets[leg_name] = [0.0, 0.0, 0.0]
            # Subscribe to gait offsets
            self.create_subscription(Point, f'/hexapod/{leg_name}/gait_offset', 
                                     lambda msg, name=leg_name: self.gait_callback(msg, name), 10)
            
        # 4. Subscriber for body pose
        self.create_subscription(Pose, '/hexapod/body_pose', self.pose_callback, 10)
        
        # 5. Timer for smooth updates (50 Hz)
        self.create_timer(0.02, self.update_loop)
        
        self.get_logger().info('Hexapod Body Kinematics Node with Gait Support initialized.')

    def gait_callback(self, msg, name):
        self.gait_offsets[name] = [msg.x, msg.y, msg.z]

    def pose_callback(self, msg):
        # Convert Quaternion to Euler
        q = msg.orientation
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp) # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        target = [msg.position.x, msg.position.y, msg.position.z, roll, pitch, yaw]
        self.pose_interp.set_target(target)

    def update_loop(self):
        # 1. Get smoothed current pose
        curr_pose = self.pose_interp.update()
        
        translation = curr_pose[0:3]
        rotation = curr_pose[3:6]
        
        # 2. Calculate IK
        results = self.body_ik.calculate_body_ik(translation, rotation)
        
        # 3. Publish leg targets (Base + Gait offset)
        for leg_name, pos in results.items():
            p_msg = Point()
            # Sum body IK result and gait offset
            g_off = self.gait_offsets.get(leg_name, [0.0, 0.0, 0.0])
            p_msg.x = pos['x'] + g_off[0]
            p_msg.y = pos['y'] + g_off[1]
            p_msg.z = pos['z'] + g_off[2]
            self.leg_pubs[leg_name].publish(p_msg)

def main(args=None):
    rclpy.init(args=args)
    node = HexapodBodyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()