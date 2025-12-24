import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String, Bool
import time

# Try to import the hardware library
try:
    import board
    import busio
    from adafruit_mcp230xx.mcp23017 import MCP23017
    import digitalio
    HAS_HARDWARE = True
except ImportError:
    HAS_HARDWARE = False

class MCP23017Node(Node):
    """
    ROS2 Node for the MCP23017 16-channel I2C GPIO Expander.
    Supports individual pin control and reading the entire port state.
    """

    def __init__(self):
        super().__init__('mcp23017_node')

        # Parameters
        self.declare_parameter('i2c_address', 0x20)
        self.declare_parameter('publish_rate', 10.0) # Hz for inputs
        self.declare_parameter('mock_hardware', not HAS_HARDWARE)

        addr = self.get_parameter('i2c_address').value
        self.rate = self.get_parameter('publish_rate').value
        self.mock_mode = self.get_parameter('mock_hardware').value

        # Hardware Setup
        self.pins = []
        if self.mock_mode:
            self.get_logger().warn('MCP23017: Running in MOCK mode.')
        else:
            try:
                i2c = busio.I2C(board.SCL, board.SDA)
                self.mcp = MCP23017(i2c, address=addr)
                
                # Initialize all 16 pins as inputs with pull-ups by default
                for i in range(16):
                    pin = self.mcp.get_pin(i)
                    pin.direction = digitalio.Direction.INPUT
                    pin.pull = digitalio.Pull.UP
                    self.pins.append(pin)
                
                self.get_logger().info(f'MCP23017 initialized at 0x{addr:02X}')
            except Exception as e:
                self.get_logger().error(f'Failed to initialize MCP23017: {e}')
                self.mock_mode = True

        # Publishers
        self.input_pub = self.create_publisher(Int32, '~/inputs', 10)

        # Subscribers
        self.output_sub = self.create_subscription(String, '~/set_output', self.output_callback, 10)

        # Timer
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)

    def output_callback(self, msg):
        """Sets a pin to output mode and changes its state. Format: 'pin:state' (e.g. '0:1')"""
        try:
            parts = msg.data.split(':')
            pin_idx = int(parts[0])
            state = bool(int(parts[1]))

            if not (0 <= pin_idx <= 15):
                self.get_logger().error(f"Invalid pin index: {pin_idx}")
                return

            if self.mock_mode:
                self.get_logger().info(f"MOCK: Set Pin {pin_idx} to {state}")
            else:
                pin = self.pins[pin_idx]
                pin.direction = digitalio.Direction.OUTPUT
                pin.value = state
                self.get_logger().debug(f"Set Pin {pin_idx} to {state}")

        except Exception as e:
            self.get_logger().error(f"Error setting output: {e}")

    def timer_callback(self):
        """Reads all pins and publishes as a bitmask."""
        try:
            mask = 0
            if self.mock_mode:
                mask = int(time.time()) % 65536
            else:
                for i in range(16):
                    # If pin is input, read its value
                    if self.pins[i].direction == digitalio.Direction.INPUT:
                        if self.pins[i].value:
                            mask |= (1 << i)
            
            msg = Int32()
            msg.data = mask
            self.input_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Error reading inputs: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MCP23017Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
