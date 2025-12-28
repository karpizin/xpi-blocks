#!/usr/bin/env python3
import sys
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Quaternion
import yaml

# Добавляем пути
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from body_kinematics import BodyKinematics

class HexapodBodyNode(Node):
    def __init__(self):
        super().__init__('hexapod_body_node')
        
        # 1. Загрузка конфига
        config_path = '/Users/slava/Documents/xpi-blocks/projects/hexapod_master/config/hexapod_config.yaml'
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        
        self.body_ik = BodyKinematics(self.config['legs'])
        
        # 2. Publishers для каждой лапы
        self.leg_pubs = {}
        for leg_name in self.config['legs'].keys():
            topic = f'/hexapod/{leg_name}/goal_point'
            self.leg_pubs[leg_name] = self.create_publisher(Point, topic, 10)
            
        # 3. Subscriber для позы корпуса
        self.create_subscription(Pose, '/hexapod/body_pose', self.pose_callback, 10)
        
        self.get_logger().info('Hexapod Body Kinematics Node initialized.')

    def pose_callback(self, msg):
        # Конвертируем Quaternion в Euler (упрощенно)
        # Для прототипа используем x,y,z смещение и напрямую углы если нужно
        # Но по стандарту Pose использует кватернионы
        
        translation = (msg.position.x, msg.position.y, msg.position.z)
        # Временная заглушка для углов (будем брать из ориентации позже)
        rotation = (0.0, 0.0, 0.0) 
        
        results = self.body_ik.calculate_body_ik(translation, rotation)
        
        for leg_name, pos in results.items():
            p_msg = Point()
            p_msg.x = pos['x']
            p_msg.y = pos['y']
            p_msg.z = pos['z']
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
