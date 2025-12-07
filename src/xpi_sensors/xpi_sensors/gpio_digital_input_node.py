import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int8MultiArray
from gpiozero import InputDevice, Device
from gpiozero.pins.mock import MockFactory
import os
import time

class GPIODigitalInputNode(Node):
    """
    ROS2 Node for reading multiple digital input devices (e.g., switches, PIR, flame sensors).
    Publishes boolean state for each configured GPIO pin.
    """

    def __init__(self):
        super().__init__('gpio_digital_input_node')

        # 1. Declare Parameters
        self.declare_parameter('pins', [18, 23]) # List of BCM GPIO pins
        self.declare_parameter('pull_up', True)  # Use internal pull-up resistor
        self.declare_parameter('pull_down', False) # Use internal pull-down resistor
        self.declare_parameter('publish_rate', 10.0) # Hz, how often to check/publish state
        self.declare_parameter('mock_hardware', False) # For testing without real GPIO

        # 2. Read Parameters
        self.pins = self.get_parameter('pins').value
        self.pull_up = self.get_parameter('pull_up').value
        self.pull_down = self.get_parameter('pull_down').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.mock_mode = self.get_parameter('mock_hardware').value

        # Validate pull_up/pull_down (only one can be true)
        if self.pull_up and self.pull_down:
            self.get_logger().error("Both pull_up and pull_down cannot be true. Defaulting to pull_up=True, pull_down=False.")
            self.pull_down = False

        # Initialize mock factory if needed
        if self.mock_mode or os.environ.get('GPIOZERO_PIN_FACTORY') == 'mock':
            self.get_logger().warn('DigitalInput: Running in MOCK mode. No real GPIO will be used.')
            Device.pin_factory = MockFactory()
            
        self.input_devices = {}
        self.publishers = {} # Publishers for individual pins
        self.multi_pin_publisher = None # Publisher for all pins in one message

        try:
            for pin in self.pins:
                self.input_devices[pin] = InputDevice(pin, pull_up=self.pull_up, pull_down=self.pull_down)
                self.publishers[pin] = self.create_publisher(Bool, f'~/pin_{pin}/state', 10)
                self.get_logger().info(f'DigitalInput: Initialized GPIO {pin} (pull_up={self.pull_up}, pull_down={self.pull_down}).')
            
            # Create a publisher for all pins combined
            if len(self.pins) > 1:
                self.multi_pin_publisher = self.create_publisher(Int8MultiArray, '~/all_pins_state', 10)

        except Exception as e:
            self.get_logger().error(f'DigitalInput: Failed to initialize GPIO pins: {e}')
            self.mock_mode = True # Fallback to mock if real GPIO fails
            # Create dummy devices for mock mode if not already done
            for pin in self.pins:
                self.input_devices[pin] = MockInputDevice(pin)


        # 3. Timer for checking/publishing states
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        self.get_logger().info(f'DigitalInput: Publishing states at {self.publish_rate} Hz.')

    def timer_callback(self):
        """Checks pin states and publishes messages."""
        all_pin_states = []
        for pin in self.pins:
            state = self.input_devices[pin].is_active
            
            # Publish individual pin state
            msg = Bool()
            msg.data = state
            self.publishers[pin].publish(msg)
            
            all_pin_states.append(1 if state else 0)

        # Publish all pin states in one message
        if self.multi_pin_publisher and all_pin_states:
            multi_msg = Int8MultiArray()
            multi_msg.data = all_pin_states
            self.multi_pin_publisher.publish(multi_msg)

    def destroy_node(self):
        # Close GPIO devices
        for pin, device in self.input_devices.items():
            device.close()
            self.get_logger().info(f'DigitalInput: Closed GPIO {pin}.')
        
        super().destroy_node()

# Simple MockInputDevice for when GPIO initialization fails in mock mode
class MockInputDevice:
    def __init__(self, pin=None, pull_up=True, pull_down=False):
        self.pin = pin
        self._is_active = False
        # Simulate some activity for mock
        self._last_toggle_time = time.monotonic()
        self._toggle_interval = 2.0 # Toggle every 2 seconds for mock
        # logging.getLogger(__name__).debug(f'MockInputDevice {pin} initialized')

    @property
    def is_active(self):
        if (time.monotonic() - self._last_toggle_time) > self._toggle_interval:
            self._is_active = not self._is_active
            self._last_toggle_time = time.monotonic()
            # logging.getLogger(__name__).debug(f'MockInputDevice {self.pin} toggled to {self._is_active}')
        return self._is_active

    def close(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = GPIODigitalInputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
