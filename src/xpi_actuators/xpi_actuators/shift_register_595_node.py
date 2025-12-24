import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from gpiozero import DigitalOutputDevice, Device
from gpiozero.pins.mock import MockFactory
import time

class ShiftRegister595Node(Node):
    """
    ROS2 Node for 74HC595 Shift Register.
    Converts a serial input into 8 parallel outputs.
    """

    def __init__(self):
        super().__init__('shift_register_595_node')

        # Parameters
        self.declare_parameter('data_pin', 17)
        self.declare_parameter('clock_pin', 27)
        self.declare_parameter('latch_pin', 22)
        self.declare_parameter('mock_hardware', False)

        data_p = self.get_parameter('data_pin').value
        clock_p = self.get_parameter('clock_pin').value
        latch_p = self.get_parameter('latch_pin').value
        mock_mode = self.get_parameter('mock_hardware').value

        if mock_mode:
            Device.pin_factory = MockFactory()
            self.get_logger().warn('74HC595: Running in MOCK mode.')

        # GPIO Setup
        self.ds = DigitalOutputDevice(data_p)
        self.sh_cp = DigitalOutputDevice(clock_p)
        self.st_cp = DigitalOutputDevice(latch_p)

        # Subscriber
        self.create_subscription(Int32, '~/raw_mask', self.mask_callback, 10)
        
        self.get_logger().info(f"74HC595 initialized on Pins: DS={data_p}, Clock={clock_p}, Latch={latch_p}")

    def mask_callback(self, msg):
        mask = msg.data & 0xFF # We only handle 8 bits for now
        self.shift_out(mask)

    def shift_out(self, val):
        """Standard bit-banging for 74HC595."""
        self.st_cp.off() # Latch LOW
        
        for i in range(8):
            # Send bit (MSB first)
            bit = (val >> (7 - i)) & 0x01
            self.ds.value = bool(bit)
            
            # Pulse Clock
            self.sh_cp.on()
            self.sh_cp.off()
            
        self.st_cp.on() # Latch HIGH (Update outputs)
        self.get_logger().debug(f"Shifted out: {bin(val)}")

def main(args=None):
    rclpy.init(args=args)
    node = ShiftRegister595Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
