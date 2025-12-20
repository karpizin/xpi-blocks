#!/usr/bin/env python3
"""
Simple Swarm Controller.
Adjusts local robot instructions based on neighbor telemetry.
"""
import rclpy
from rclpy.node import Node
import json
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist # Для управления скоростью/курсом

class SwarmControllerNode(Node):
    def __init__(self):
        super().__init__('swarm_controller')

        # 1. Параметры
        self.declare_parameter('safe_distance', 5.0) # метры
        self.safe_distance = self.get_parameter('safe_distance').value

        # 2. Подписки
        self.sub_neighbors = self.create_subscription(String, '/meshtastic_bridge/neighbors', self._neighbors_callback, 10)
        self.sub_local_gps = self.create_subscription(String, '/gps/state', self._local_gps_callback, 10) # Упрощенно

        # 3. Публикация управления
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        self.local_pos = None
        self.get_logger().info("Swarm Controller started. Monitoring neighbors for safety...")

    def _local_gps_callback(self, msg):
        self.local_pos = json.loads(msg.data)

    def _neighbors_callback(self, msg):
        if not self.local_pos:
            return

        neighbor_data = json.loads(msg.data)
        n_pos = neighbor_data.get('data', {})
        
        # Рассчитываем дистанцию (упрощенно, в градусах переведенных в метры)
        # В реальности нужно использовать Haversine formula
        dist = self._calculate_distance(
            self.local_pos.get('lat'), self.local_pos.get('lon'),
            n_pos.get('lat'), n_pos.get('lon')
        )

        if dist < self.safe_distance:
            self.get_logger().warn(f"COLLISION ALERT! Neighbor {neighbor_data['node_id']} is too close ({dist:.2f}m)")
            self._apply_avoidance_logic()

    def _calculate_distance(self, lat1, lon1, lat2, lon2):
        # Упрощенная формула для малых расстояний
        if None in [lat1, lon1, lat2, lon2]: return 1000.0
        R = 6371000 # Радиус Земли в метрах
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
        return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1-a))

    def _apply_avoidance_logic(self):
        """Отправляет команду на остановку или отворот."""
        msg = Twist()
        msg.linear.x = -0.5 # Назад
        msg.angular.z = 1.0 # Поворот
        self.pub_cmd_vel.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SwarmControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
