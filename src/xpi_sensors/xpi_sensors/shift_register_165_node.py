import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from gpiozero import DigitalOutputDevice, DigitalInputDevice, Device
from gpiozero.pins.mock import MockFactory
import time

class ShiftRegister165Node(Node):
    """
    ROS2 Node for 74HC165 Shift Register.
    Reads 8 parallel inputs and sends them as a serial stream.
    """

    def __init__(self):
        super().__init__('shift_register_165_node')

        # Parameters
        self.declare_parameter('load_pin', 23)
        self.declare_parameter('clock_pin', 24)
        self.declare_parameter('data_pin', 25)
        self.declare_parameter('publish_rate', 10.0) # Hz
        self.declare_parameter('mock_hardware', False)

        load_p = self.get_parameter('load_pin').value
        clock_p = self.get_parameter('clock_pin').value
        data_p = self.get_parameter('data_pin').value
        self.rate = self.get_parameter('publish_rate').value
        mock_mode = self.get_parameter('mock_hardware').value

        if mock_mode:
            Device.pin_factory = MockFactory()
            self.get_logger().warn('74HC165: Running in MOCK mode.')

        # GPIO Setup
        self.pl = DigitalOutputDevice(load_p)
        self.cp = DigitalOutputDevice(clock_p)
        self.q7 = DigitalInputDevice(data_p)

        # Publisher
        self.pub = self.create_publisher(Int32, '~/raw_mask', 10)

        # Timer
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)
        
        self.get_logger().info(f"74HC165 initialized on Pins: Load={load_p}, Clock={clock_p}, Data={data_p}")

    def timer_callback(self):
        mask = self.shift_in()
        msg = Int32()
        msg.data = mask
        self.pub.publish(msg)

    def shift_in(self):
        """Bit-banging for 74HC165."""
        if Device.pin_factory.__class__.__name__ == 'MockFactory':
            return int(time.time()) % 256 # Just dummy data

        # Pulse Load (Parallel Load active LOW)
        self.pl.off()
        time.sleep(0.001)
        self.pl.on()

        val = 0
        for i in range(8):
            # Read bit (MSB first)
            if self.q7.value:
                val |= (1 << (7 - i))
            
            # Pulse Clock
            self.cp.on()
            self.cp.off()
            
        return val

def main(args=None):
    rclpy.init(args=args)
    node = ShiftRegister165Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
