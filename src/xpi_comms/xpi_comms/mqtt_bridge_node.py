#!/usr/bin/env python3
import time
import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MQTTBridgeNode(Node):
    def __init__(self):
        super().__init__('mqtt_bridge_node')
        
        # Parameters
        self.declare_parameter('broker_address', 'localhost')
        self.declare_parameter('broker_port', 1883)
        self.declare_parameter('username', '')
        self.declare_parameter('password', '')
        self.declare_parameter('mqtt_topic_sub', 'robot/rx') # Listen on MQTT, pub to ROS
        self.declare_parameter('mqtt_topic_pub', 'robot/tx') # Listen on ROS, pub to MQTT

        self.broker = self.get_parameter('broker_address').value
        self.port = self.get_parameter('broker_port').value
        self.user = self.get_parameter('username').value
        self.pw = self.get_parameter('password').value
        self.topic_sub = self.get_parameter('mqtt_topic_sub').value
        self.topic_pub = self.get_parameter('mqtt_topic_pub').value

        # ROS Setup
        self.ros_sub = self.create_subscription(String, '~/tx', self.ros_tx_callback, 10)
        self.ros_pub = self.create_publisher(String, '~/rx', 10)

        # MQTT Setup
        self.client = mqtt.Client()
        if self.user and self.pw:
            self.client.username_pw_set(self.user, self.pw)
            
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        
        try:
            self.client.connect(self.broker, self.port, 60)
            self.client.loop_start() # Background thread
            self.get_logger().info(f"Connecting to MQTT Broker: {self.broker}:{self.port}")
        except Exception as e:
            self.get_logger().error(f"MQTT Connection Failed: {e}")

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("Connected to MQTT Broker!")
            self.client.subscribe(self.topic_sub)
            self.get_logger().info(f"Subscribed to MQTT topic: {self.topic_sub}")
        else:
            self.get_logger().error(f"Failed to connect, return code {rc}")

    def on_message(self, client, userdata, msg):
        # MQTT -> ROS
        try:
            payload_str = msg.payload.decode('utf-8')
            ros_msg = String()
            ros_msg.data = payload_str
            self.ros_pub.publish(ros_msg)
            # self.get_logger().debug(f"MQTT->ROS: {payload_str}")
        except Exception as e:
            self.get_logger().warn(f"Failed to process MQTT message: {e}")

    def ros_tx_callback(self, msg):
        # ROS -> MQTT
        try:
            self.client.publish(self.topic_pub, msg.data)
            # self.get_logger().debug(f"ROS->MQTT: {msg.data}")
        except Exception as e:
            self.get_logger().warn(f"Failed to publish to MQTT: {e}")

    def destroy_node(self):
        self.client.loop_stop()
        self.client.disconnect()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MQTTBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
