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
from interpolator import Interpolator

class HexapodBodyNode(Node):
    def __init__(self):
        super().__init__('hexapod_body_node')
        
        # 1. Загрузка конфига
        config_path = '/Users/slava/Documents/xpi-blocks/projects/hexapod_master/config/hexapod_config.yaml'
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        
        self.body_ik = BodyKinematics(self.config['legs'])
        
        # 2. Интерполятор для позы (x, y, z, roll, pitch, yaw)
        # Начальная высота берется из конфига
        h = self.config.get('default_height', 0.08)
        self.pose_interp = Interpolator([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], speed=0.05) # 5см или 0.05рад в сек
        
        # 3. Publishers для каждой лапы
        self.leg_pubs = {}
        for leg_name in self.config['legs'].keys():
            topic = f'/hexapod/{leg_name}/goal_point'
            self.leg_pubs[leg_name] = self.create_publisher(Point, topic, 10)
            
        # 4. Subscriber для позы корпуса
        self.create_subscription(Pose, '/hexapod/body_pose', self.pose_callback, 10)
        
        # 5. Таймер для плавного обновления (50 Гц)
        self.create_timer(0.02, self.update_loop)
        
        self.get_logger().info('Hexapod Body Kinematics Node with Smoothing initialized.')

    def pose_callback(self, msg):
        # Устанавливаем новую цель для интерполятора
        # Для простоты берем x,y,z и r,p,y (имитируем из кватерниона)
        target = [msg.position.x, msg.position.y, msg.position.z, 0.0, 0.0, 0.0]
        self.pose_interp.set_target(target)

    def update_loop(self):
        # 1. Получаем сглаженную текущую позу
        curr_pose = self.pose_interp.update()
        
        translation = curr_pose[0:3]
        rotation = curr_pose[3:6]
        
        # 2. Считаем IK
        results = self.body_ik.calculate_body_ik(translation, rotation)
        
        # 3. Публикуем цели для лап
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
