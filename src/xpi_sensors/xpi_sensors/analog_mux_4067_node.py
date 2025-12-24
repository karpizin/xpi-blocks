import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from gpiozero import DigitalOutputDevice, Device
from gpiozero.pins.mock import MockFactory
import time

class AnalogMux4067Node(Node):
    """
    ROS2 Node for CD74HC4067 16-channel analog multiplexer.
    Controls 4 selection pins (S0-S3) to route one of 16 inputs to a common SIG pin.
    """

    def __init__(self):
        super().__init__('analog_mux_4067_node')

        # Parameters
        self.declare_parameter('s0_pin', 17)
        self.declare_parameter('s1_pin', 27)
        self.declare_parameter('s2_pin', 22)
        self.declare_parameter('s3_pin', 10)
        self.declare_parameter('initial_channel', 0)
        self.declare_parameter('mock_hardware', False)

        pins = [
            self.get_parameter('s0_pin').value,
            self.get_parameter('s1_pin').value,
            self.get_parameter('s2_pin').value,
            self.get_parameter('s3_pin').value
        ]
        
        mock_mode = self.get_parameter('mock_hardware').value
        if mock_mode:
            Device.pin_factory = MockFactory()
            self.get_logger().warn('AnalogMux 4067: Running in MOCK mode.')

        # GPIO Setup
        self.s_pins = [DigitalOutputDevice(p) for p in pins]
        
        # State
        self.current_channel = -1
        
        # Subscriber for channel selection
        self.create_subscription(Int32, '~/select', self.select_callback, 10)
        
        # Publisher for status
        self.status_pub = self.create_publisher(Int32, '~/current_channel', 10)

        # Initialize
        self.set_channel(self.get_parameter('initial_channel').value)
        self.get_logger().info(f"CD74HC4067 initialized on Pins: S0-S3={pins}")

    def select_callback(self, msg):
        if 0 <= msg.data <= 15:
            self.set_channel(msg.data)
        else:
            self.get_logger().error(f"Invalid channel requested: {msg.data}. Must be 0-15.")

    def set_channel(self, channel):
        if channel == self.current_channel:
            return

        # Set binary pattern on S0-S3
        for i in range(4):
            bit = (channel >> i) & 0x01
            self.s_pins[i].value = bool(bit)
        
        self.current_channel = channel
        
        # Publish current channel
        status_msg = Int32()
        status_msg.data = channel
        self.status_pub.publish(status_msg)
        
        self.get_logger().info(f"Switched to analog channel {channel} (Binary: {bin(channel)[2:].zfill(4)})")

def main(args=None):
    rclpy.init(args=args)
    node = AnalogMux4067Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
