import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Int32
import serial
import time

class TFminiPlusNode(Node):
    """
    ROS2 Node for Benewake TFmini Plus LiDAR rangefinder.
    Communicates via UART.
    """

    def __init__(self):
        super().__init__('tfmini_plus_node')

        # Parameters
        self.declare_parameter('port', '/dev/ttyS0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('publish_rate', 20.0) # Hz
        self.declare_parameter('frame_id', 'tfmini_plus_link')
        self.declare_parameter('mock_hardware', False)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        self.rate = self.get_parameter('publish_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.mock_mode = self.get_parameter('mock_hardware').value

        # Hardware Setup
        if self.mock_mode:
            self.get_logger().warn('TFmini Plus: Running in MOCK mode.')
        else:
            try:
                self.ser = serial.Serial(port, baud, timeout=0.1)
                self.get_logger().info(f'TFmini Plus initialized on {port} at {baud} bps.')
            except Exception as e:
                self.get_logger().error(f'Failed to open serial port {port}: {e}')
                self.mock_mode = True

        # Publishers
        self.range_pub = self.create_publisher(Range, '~/range', 10)
        self.strength_pub = self.create_publisher(Int32, '~/strength', 10)

        # Timer
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)

    def read_tfmini(self):
        """Reads one 9-byte packet from TFmini Plus."""
        if self.mock_mode:
            import random
            return 2.5 + random.uniform(-0.05, 0.05), 1000

        # Wait for frame header (0x59 0x59)
        while self.ser.in_waiting >= 9:
            if self.ser.read(1) == b'\x59':
                if self.ser.read(1) == b'\x59':
                    # Header found, read remaining 7 bytes
                    data = self.ser.read(7)
                    if len(data) < 7: break
                    
                    dist_l = data[0]
                    dist_h = data[1]
                    str_l = data[2]
                    str_h = data[3]
                    
                    # Calculate distance in cm
                    distance = (dist_h << 8) | dist_l
                    # Calculate strength
                    strength = (str_h << 8) | str_l
                    
                    return distance / 100.0, strength # m, strength
        return None, None

    def timer_callback(self):
        dist, strength = self.read_tfmini()
        
        if dist is not None:
            # Publish Range
            msg = Range()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.radiation_type = Range.INFRARED
            msg.field_of_view = 0.063 # 3.6 degrees in radians (narrow beam)
            msg.min_range = 0.1
            msg.max_range = 12.0
            msg.range = float(dist)
            self.range_pub.publish(msg)

            # Publish Strength
            s_msg = Int32()
            s_msg.data = int(strength)
            self.strength_pub.publish(s_msg)
            
            self.get_logger().debug(f'TFmini Range: {dist:.2f}m, Strength: {strength}')

    def destroy_node(self):
        if not self.mock_mode and hasattr(self, 'ser'):
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TFminiPlusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
