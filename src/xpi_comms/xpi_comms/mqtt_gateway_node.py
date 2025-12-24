import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Int32, Bool
import paho.mqtt.client as mqtt
import json
import os

class MQTTGatewayNode(Node):
    """
    Advanced ROS2 to MQTT Gateway.
    Supports multiple topics, JSON packaging, and various data types.
    """

    def __init__(self):
        super().__init__('mqtt_gateway_node')

        # 1. Parameters
        self.declare_parameter('broker_address', 'localhost')
        self.declare_parameter('broker_port', 1883)
        self.declare_parameter('client_id', 'xpi_robot')
        self.declare_parameter('topics_to_mqtt', []) # List of ROS topics to publish to MQTT
        self.declare_parameter('mqtt_prefix', 'xpi_robot/') # Prefix for all MQTT topics

        self.broker = self.get_parameter('broker_address').value
        self.port = self.get_parameter('broker_port').value
        self.client_id = self.get_parameter('client_id').value
        self.topics_to_mqtt = self.get_parameter('topics_to_mqtt').value
        self.prefix = self.get_parameter('mqtt_prefix').value

        # 2. MQTT Setup
        self.mqtt_client = mqtt.Client(client_id=self.client_id)
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        
        try:
            self.mqtt_client.connect(self.broker, self.port, 60)
            self.mqtt_client.loop_start()
            self.get_logger().info(f"Connecting to MQTT Broker: {self.broker}:{self.port}")
        except Exception as e:
            self.get_logger().error(f"MQTT Connection Failed: {e}")

        # 3. Dynamic ROS Subscriptions (ROS -> MQTT)
        self.subs = []
        for topic in self.topics_to_mqtt:
            # We subscribe to any message type and convert to JSON
            # For simplicity in this block, we assume common types or Strings
            sub = self.create_subscription(
                String, # Default to String for raw proxying
                topic,
                lambda msg, t=topic: self.ros_to_mqtt_callback(msg, t),
                10
            )
            self.subs.append(sub)
            self.get_logger().info(f"Gateway: Forwarding ROS topic {topic} to MQTT")

        # 4. JSON Command Subscriber (MQTT -> ROS)
        # We listen to a single command topic on MQTT and dispatch to ROS
        self.cmd_topic = f"{self.prefix}cmd"
        
        # Publisher for MQTT responses into ROS
        self.ros_pub = self.create_publisher(String, '~/mqtt_rx', 10)

    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("Connected to MQTT Broker!")
            client.subscribe(self.cmd_topic)
            self.get_logger().info(f"Listening for MQTT commands on: {self.cmd_topic}")
        else:
            self.get_logger().error(f"MQTT Connection failed with code {rc}")

    def on_mqtt_message(self, client, userdata, msg):
        """Dispatches MQTT messages to internal ROS topics."""
        try:
            payload = msg.payload.decode('utf-8')
            self.get_logger().info(f"MQTT Command received: {payload}")
            
            # Simple dispatch logic: if it's JSON, we could route it
            # For now, just publish to internal RX topic
            ros_msg = String()
            ros_msg.data = payload
            self.ros_pub.publish(ros_msg)
            
        except Exception as e:
            self.get_logger().warn(f"Failed to process MQTT message: {e}")

    def ros_to_mqtt_callback(self, msg, topic_name):
        """Forwards ROS messages to MQTT with prefix."""
        mqtt_topic = self.prefix + topic_name.strip('/')
        try:
            # Try to see if it's a standard message with .data
            data = msg.data if hasattr(msg, 'data') else str(msg)
            
            # Pack into JSON for better IoT integration
            payload = json.dumps({
                "source": "ros2",
                "topic": topic_name,
                "value": data,
                "timestamp": time.time()
            })
            
            self.mqtt_client.publish(mqtt_topic, payload)
        except Exception as e:
            self.get_logger().error(f"Failed to forward {topic_name}: {e}")

    def destroy_node(self):
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MQTTGatewayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
