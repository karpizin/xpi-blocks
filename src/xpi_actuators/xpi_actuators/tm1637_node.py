import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import os
import time

try:
    import tm1637
    # For mocking
except ImportError:
    class MockTM1637:
        def __init__(self, clk, dio, brightness=7):
            self.clk = clk
            self.dio = dio
            self.brightness = brightness
            self.segments = [0] * 4 # Assuming 4-digit display
            self.logger = logging.getLogger('MockTM1637')
            self.logger.info(f"MockTM1637 initialized. CLK:{clk}, DIO:{dio}")
        def write(self, segments, colon=False):
            self.segments = segments
            self.logger.info(f"MockTM1637: Displaying segments {segments}, colon: {colon}")
        def numbers(self, num1, num2=None, colon=False):
            self.logger.info(f"MockTM1637: Displaying numbers {num1}, {num2}, colon: {colon}")
        def temperature(self, temp, colon=False):
            self.logger.info(f"MockTM1637: Displaying temperature {temp}, colon: {colon}")
        def hex(self, digits):
            self.logger.info(f"MockTM1637: Displaying hex {digits}")
        def brightness(self, brightness):
            self.brightness = brightness
            self.logger.info(f"MockTM1637: Set brightness to {brightness}")
    tm1637 = MockTM1637 # Assign mock class

class TM1637Node(Node):
    """
    ROS2 Node for controlling a TM1637 4/6-digit 7-segment display.
    Supports displaying text/numbers and setting brightness.
    """

    def __init__(self):
        super().__init__('tm1637_node')

        # 1. Declare Parameters
        self.declare_parameter('clk_pin', 23)        # GPIO pin for CLK.
        self.declare_parameter('dio_pin', 24)        # GPIO pin for DIO.
        self.declare_parameter('brightness', 7)      # Brightness (0-7).
        self.declare_parameter('mock_hardware', False) # For testing without real hardware.
        self.declare_parameter('display_digits', 4)  # Number of digits on display (4 or 6).

        # 2. Read Parameters
        self.clk_pin = self.get_parameter('clk_pin').value
        self.dio_pin = self.get_parameter('dio_pin').value
        self.brightness = self.get_parameter('brightness').value
        self.mock_mode = self.get_parameter('mock_hardware').value
        self.display_digits = self.get_parameter('display_digits').value

        self.display = None
        if not self.mock_mode:
            try:
                # Initialize TM1637 display
                # Note: tm1637 library usually requires root access if using native GPIO.
                # It can sometimes work without, depending on wiringPi/pigpio setup.
                self.display = tm1637.TM1637(
                    clk=self.clk_pin, 
                    dio=self.dio_pin, 
                    brightness=self.brightness
                )
                self.display.clear()
                self.get_logger().info(f'TM1637: Initialized on CLK:{self.clk_pin}, DIO:{self.dio_pin}.')
            except Exception as e:
                self.get_logger().error(f'TM1637: Failed to initialize TM1637 display: {e}. '
                                        'Ensure you have `tm1637` installed and GPIOs configured. '
                                        'Falling back to mock mode.')
                self.mock_mode = True
        
        if self.mock_mode:
            self.get_logger().warn('TM1637: Running in MOCK mode. No real display will be controlled.')
            self.display = tm1637.TM1637(clk=self.clk_pin, dio=self.dio_pin, brightness=self.brightness)

        # 3. Subscribers
        qos_profile = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)

        self.text_sub = self.create_subscription(
            String,
            '~/display_text',
            self.display_text_callback,
            qos_profile
        )
        self.brightness_sub = self.create_subscription(
            UInt8,
            '~/set_brightness',
            self.set_brightness_callback,
            qos_profile
        )
        self.get_logger().info(f'TM1637: Subscribing to display commands.')

        # Display initial text
        self.display_text_callback(String(data=self.default_text))


    def display_text_callback(self, msg: String):
        """Displays text/numbers on the TM1637 display."""
        text = str(msg.data)
        if self.display:
            # The tm1637 library handles converting string to segments
            # for 4 or 6 digits.
            self.display.show(text)
            self.get_logger().debug(f"TM1637: Displayed text: '{text}'")

    def set_brightness_callback(self, msg: UInt8):
        """Sets the brightness of the display."""
        brightness = min(max(0, msg.data), 7) # Brightness is 0-7
        if self.display:
            self.display.brightness(brightness)
            self.get_logger().info(f"TM1637: Set brightness to {brightness}.")


    def destroy_node(self):
        if self.display:
            self.display.clear() # Clear display on shutdown
            self.get_logger().info('TM1637: Display cleared.')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TM1637Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
