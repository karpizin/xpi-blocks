#!/usr/bin/env python3
import json
import can
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CANBridgeNode(Node):
    def __init__(self):
        super().__init__('can_bridge_node')
        
        # Parameters
        self.declare_parameter('interface', 'can0')
        self.declare_parameter('bitrate', 500000)
        
        self.iface = self.get_parameter('interface').value
        self.bitrate = self.get_parameter('bitrate').value

        # ROS Setup
        self.tx_sub = self.create_subscription(String, '~/tx', self.ros_tx_callback, 10)
        self.rx_pub = self.create_publisher(String, '~/rx', 10)

        # CAN Setup
        try:
            self.bus = can.interface.Bus(channel=self.iface, bustype='socketcan', bitrate=self.bitrate)
            
            # Start a listener thread for RX
            self.notifier = can.Notifier(self.bus, [self.can_rx_callback])
            
            self.get_logger().info(f"CAN Bridge initialized on {self.iface} at {self.bitrate} bps")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize CAN bus: {e}")
            self.bus = None

    def can_rx_callback(self, msg):
        """Called when a frame is received on the CAN bus."""
        try:
            frame_data = {
                "id": msg.arbitration_id,
                "data": list(msg.data),
                "dlc": msg.dlc,
                "is_extended": msg.is_extended_id,
                "is_remote": msg.is_remote_frame,
                "is_error": msg.is_error_frame
            }
            ros_msg = String()
            ros_msg.data = json.dumps(frame_data)
            self.rx_pub.publish(ros_msg)
        except Exception as e:
            self.get_logger().warn(f"Error processing received CAN frame: {e}")

    def ros_tx_callback(self, msg):
        """Called when a JSON string is received from ROS to be sent to CAN."""
        if not self.bus:
            return
            
        try:
            data = json.loads(msg.data)
            can_id = data.get("id")
            payload = data.get("data", [])
            is_extended = data.get("is_extended", False)
            
            can_msg = can.Message(
                arbitration_id=can_id,
                data=bytearray(payload),
                is_extended_id=is_extended
            )
            
            self.bus.send(can_msg)
            # self.get_logger().debug(f"Sent CAN frame ID: {can_id}")
        except Exception as e:
            self.get_logger().warn(f"Failed to send CAN frame: {e}")

    def destroy_node(self):
        if hasattr(self, 'notifier'):
            self.notifier.stop()
        if hasattr(self, 'bus') and self.bus:
            self.bus.shutdown()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CANBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
