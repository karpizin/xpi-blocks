#!/usr/bin/env python3
"""
ROS2 Node for Meshtastic Bridge.
Connects the XPI robot event bus with the LoRa Mesh network.
"""
import rclpy
from rclpy.node import Node
import json
import os
import sys

# Добавляем путь к нашему драйверу
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../blocks/drivers/meshtastic'))
from driver import MeshtasticDriver

from std_msgs.msg import String, Header
from sensor_msgs.msg import NavSatFix # Для GPS телеметрии

class MeshtasticBridgeNode(Node):
    def __init__(self):
        super().__init__('meshtastic_bridge')

        # 1. Параметры
        self.declare_parameter('interface', 'serial')
        self.declare_parameter('address', '/dev/ttyUSB0')
        self.declare_parameter('node_name', 'robot_01')
        
        interface_type = self.get_parameter('interface').value
        address = self.get_parameter('address').value
        self.robot_id = self.get_parameter('node_name').value

        # 2. Инициализация драйвера
        self.driver = MeshtasticDriver(interface_type, address)
        self.driver.on_telemetry_received.append(self._on_mesh_telemetry)
        self.driver.on_command_received.append(self._on_mesh_command)
        
        try:
            self.driver.connect()
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Meshtastic: {e}")

        # 3. ROS2 Интерфейсы
        # Публикация данных от соседей
        self.pub_neighbors = self.create_publisher(String, '~/neighbors', 10)
        self.pub_commands = self.create_publisher(String, '~/inbound_commands', 10)

        # Подписка на локальную телеметрию (например, GPS)
        self.sub_gps = self.create_subscription(NavSatFix, '/gps/fix', self._gps_callback, 10)
        
        # Подписка на команды, которые нужно отправить в Mesh (например, от роевого контроллера)
        self.sub_outbound = self.create_subscription(String, '~/outbound_broadcast', self._outbound_callback, 10)

        self.get_logger().info(f"Meshtastic Bridge Node started. ID: {self.robot_id}")

    def _on_mesh_telemetry(self, telemetry):
        """Вызывается драйвером при получении данных от соседа."""
        msg = String()
        msg.data = json.dumps(telemetry)
        self.pub_neighbors.publish(msg)
        # self.get_logger().info(f"Neighbor {telemetry['node_id']} updated")

    def _on_mesh_command(self, sender_id, command):
        """Вызывается при получении команды из Mesh."""
        msg = String()
        msg.data = json.dumps({"from": sender_id, "cmd": command})
        self.pub_commands.publish(msg)
        self.get_logger().info(f"Received Mesh command from {sender_id}")

    def _gps_callback(self, msg: NavSatFix):
        """Отправка локальных координат в Mesh."""
        # Мы отправляем телеметрию раз в N секунд (чтобы не забивать эфир LoRa)
        # В реальной реализации тут нужен таймер или фильтр изменений
        payload = {
            "id": self.robot_id,
            "type": "telemetry",
            "lat": msg.latitude,
            "lon": msg.longitude,
            "alt": msg.altitude
        }
        self.driver.broadcast_telemetry(payload)

    def _outbound_callback(self, msg: String):
        """Рассылка произвольных данных из ROS в Mesh."""
        try:
            data = json.loads(msg.data)
            self.driver.broadcast_telemetry(data)
        except Exception as e:
            self.get_logger().error(f"Failed to broadcast outbound data: {e}")

    def destroy_node(self):
        self.driver.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MeshtasticBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
