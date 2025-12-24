import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Int32, Float32
import serial
import time

class TFSerialNode(Node):
    """
    Universal ROS2 Node for Benewake TF series LiDARs (TF-Luna, TFmini, TF02, TF03).
    Supports 9-byte binary protocol via UART.
    """

    def __init__(self):
        super().__init__('tf_lidar_node')

        # Parameters
        self.declare_parameter('port', '/dev/ttyS0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('model', 'tf02_pro') # luna, mini, tf02_pro, tf03
        self.declare_parameter('frame_id', 'tf_lidar_link')
        self.declare_parameter('mock_hardware', False)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        self.rate = self.get_parameter('publish_rate').value
        self.model = self.get_parameter('model').value
        self.frame_id = self.get_parameter('frame_id').value
        self.mock_mode = self.get_parameter('mock_hardware').value

        # Configure max range based on model
        self.max_range = 40.0
        if 'luna' in self.model: self.max_range = 8.0
        elif 'mini' in self.model: self.max_range = 12.0
        elif 'tf03' in self.model: self.max_range = 100.0

        # Hardware Setup
        if self.mock_mode:
            self.get_logger().warn(f'TF LiDAR ({self.model}): Running in MOCK mode.')
        else:
            try:
                self.ser = serial.Serial(port, baud, timeout=0.1)
                self.get_logger().info(f'TF LiDAR ({self.model}) initialized on {port}.')
            except Exception as e:
                self.get_logger().error(f'Failed to open serial port {port}: {e}')
                self.mock_mode = True

        # Publishers
        self.range_pub = self.create_publisher(Range, '~/range', 10)
        self.strength_pub = self.create_publisher(Int32, '~/strength', 10)
        self.temp_pub = self.create_publisher(Float32, '~/temp', 10)

        # Timer
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)

    def read_packet(self):
        """Standard 9-byte packet reader for TF series."""
        if self.mock_mode:
            import random
            return 10.0 + random.uniform(-0.1, 0.1), 1500, 35.0

        while self.ser.in_waiting >= 9:
            if self.ser.read(1) == b'\x59':
                if self.ser.read(1) == b'\x59':
                    data = self.ser.read(7)
                    if len(data) < 7: break
                    
                    # Distance: bytes 2 and 3
                    distance = (data[1] << 8) | data[0]
                    # Strength: bytes 4 and 5
                    strength = (data[3] << 8) | data[2]
                    # Temperature: bytes 6 and 7
                    temp_raw = (data[5] << 8) | data[4]
                    temperature = temp_raw / 8.0 - 256.0
                    
                    return distance / 100.0, strength, temperature
        return None, None, None

    def timer_callback(self):
        dist, strength, temp = self.read_packet()
        
        if dist is not None:
            # Publish Range
            msg = Range()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.radiation_type = Range.INFRARED
            msg.field_of_view = 0.05 # ~3 degrees
            msg.min_range = 0.1
            msg.max_range = self.max_range
            msg.range = float(dist)
            self.range_pub.publish(msg)

            # Publish Strength
            s_msg = Int32()
            s_msg.data = int(strength)
            self.strength_pub.publish(s_msg)

            # Publish Temperature
            t_msg = Float32()
            t_msg.data = float(temp)
            self.temp_pub.publish(t_msg)

    def destroy_node(self):
        if not self.mock_mode and hasattr(self, 'ser'):
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TFSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
