import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8, Int8MultiArray
from gpiozero import LED, LEDBarGraph, Device
from gpiozero.pins.mock import MockFactory
import os
import math

class LEDBarNode(Node):
    """
    ROS2 Node for controlling an LED Bar Graph display using GPIO pins.
    Supports either setting number of active LEDs or individual LED states.
    """

    def __init__(self):
        super().__init__('led_bar_node')

        # 1. Declare Parameters
        self.declare_parameter('pins', [2, 3, 4, 17, 27, 22, 10, 9]) # List of BCM GPIO pins
        self.declare_parameter('invert_logic', False) # Set to True if LED is active low
        self.declare_parameter('initial_value', 0) # Number of LEDs on initially
        self.declare_parameter('mock_hardware', False)

        # 2. Read Parameters
        self.pins = self.get_parameter('pins').value
        self.invert_logic = self.get_parameter('invert_logic').value
        self.initial_value = self.get_parameter('initial_value').value
        self.mock_mode = self.get_parameter('mock_hardware').value
        
        # Initialize mock factory if needed
        if self.mock_mode or os.environ.get('GPIOZERO_PIN_FACTORY') == 'mock':
            self.get_logger().warn('LEDBar: Running in MOCK mode. No real GPIO will be used.')
            Device.pin_factory = MockFactory()

        self.led_bar = None
        try:
            # gpiozero.LEDBarGraph requires a tuple of pins
            self.led_bar = LEDBarGraph(
                self.pins,
                active_high=not self.invert_logic,
                initial_value=self.initial_value / len(self.pins) # Convert number to normalized value
            )
            self.get_logger().info(f'LEDBar: Initialized with {len(self.pins)} LEDs on GPIOs {self.pins}.')
        except Exception as e:
            self.get_logger().error(f'LEDBar: Failed to initialize LEDBarGraph: {e}. Falling back to mock.')
            self.mock_mode = True # Fallback to mock if real GPIO fails
            # Create a mock LEDBarGraph
            self.led_bar = MockLEDBarGraph(self.pins, active_high=not self.invert_logic, initial_value=self.initial_value / len(self.pins))
        
        if self.mock_mode:
            self.get_logger().warn('LEDBar: Running in MOCK mode. No real LEDs will be controlled.')

        # 3. Subscribers
        # To set the number of LEDs to light up (0 to len(pins))
        self.num_leds_sub = self.create_subscription(
            UInt8,
            '~/set_count',
            self.set_count_callback,
            10
        )
        # To set individual LED states (0 or 1 for each LED)
        self.individual_leds_sub = self.create_subscription(
            Int8MultiArray,
            '~/set_individual',
            self.set_individual_callback,
            10
        )
        self.get_logger().info('LEDBar: Subscribing to LED bar commands.')

    def set_count_callback(self, msg: UInt8):
        """Sets the number of LEDs to light up on the bar graph."""
        count = min(max(0, msg.data), len(self.pins))
        # LEDBarGraph value property expects a float from 0.0 to 1.0
        self.led_bar.value = count / len(self.pins)
        self.get_logger().info(f'LEDBar: Set {count} LEDs ON. Normalized value: {self.led_bar.value:.2f}')

    def set_individual_callback(self, msg: Int8MultiArray):
        """Sets the state of individual LEDs based on an array."""
        if len(msg.data) != len(self.pins):
            self.get_logger().warn(f'LEDBar: Received Int8MultiArray with {len(msg.data)} elements, expected {len(self.pins)}. Ignoring.')
            return
        
        for i, state in enumerate(msg.data):
            if i >= len(self.pins): # Should not happen with length check
                break
            if state == 1:
                self.led_bar.leds[i].on()
            else:
                self.led_bar.leds[i].off()
        self.get_logger().info(f'LEDBar: Set individual LED states: {msg.data}')

    def destroy_node(self):
        if self.led_bar:
            self.led_bar.off() # Turn off all LEDs on shutdown
            self.led_bar.close() # Release GPIOs
            self.get_logger().info('LEDBar: All LEDs turned off and GPIOs released.')
        super().destroy_node()

# Simple MockLEDBarGraph for when GPIO initialization fails in mock mode
class MockLED:
    def __init__(self, pin):
        self.pin = pin
        self.is_lit = False
    def on(self):
        self.is_lit = True
        # logging.getLogger(__name__).debug(f'MockLED {self.pin} ON')
    def off(self):
        self.is_lit = False
        # logging.getLogger(__name__).debug(f'MockLED {self.pin} OFF')
    def close(self):
        pass

class MockLEDBarGraph:
    def __init__(self, pins, active_high=True, initial_value=0.0):
        self.pins = pins
        self.leds = [MockLED(p) for p in pins]
        self._value = initial_value
        self.set_leds_by_value(initial_value)
        self.get_logger().info(f'MockLEDBarGraph initialized with pins: {pins}')
    
    def get_logger(self):
        return logging.getLogger(__name__)

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, val):
        self._value = max(0.0, min(1.0, val))
        self.set_leds_by_value(self._value)

    def set_leds_by_value(self, val):
        num_on = int(val * len(self.pins))
        for i, led in enumerate(self.leds):
            if i < num_on:
                led.on()
            else:
                led.off()
        self.get_logger().debug(f'MockLEDBarGraph: Set value to {val}, {num_on} LEDs ON.')

    def off(self):
        for led in self.leds:
            led.off()

    def close(self):
        for led in self.leds:
            led.close()
        self.get_logger().debug('MockLEDBarGraph: Closed.')

def main(args=None):
    rclpy.init(args=args)
    node = LEDBarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
