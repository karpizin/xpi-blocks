import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import os
import time

try:
    from luma.core.interface.serial import spi, noop
    from luma.core.render import canvas
    from luma.led_matrix.device import max7219
except ImportError:
    class spi:
        def __init__(self, port=0, device=0, cs_high=False, bus_speed_hz=8000000, gpio=None, **kwargs):
            pass
    class noop:
        def __init__(self):
            pass
    class canvas:
        def __init__(self, device):
            self.device = device
        def __enter__(self):
            return MockDraw(self.device)
        def __exit__(self, exc_type, exc_val, exc_tb):
            pass
    class max7219:
        def __init__(self, serial_interface, cascaded=1, block_orientation=0, rotate=0, **kwargs):
            self.cascaded = cascaded
            self.width = cascaded * 8
            self.height = 8
            self.display_buffer = [(0) for _ in range(self.width * self.height)]
            self.brightness = 0
            self.contrast = 0
            self.logger = logging.getLogger('MockMAX7219')
            self.logger.info(f"MockMAX7219 initialized. Cascaded: {cascaded}")
        def show(self):
            # self.logger.debug("MockMAX7219 show called.")
            pass
        def set_pixel(self, x, y, value):
            if 0 <= x < self.width and 0 <= y < self.height:
                self.display_buffer[y * self.width + x] = value
        def clear(self):
            self.display_buffer = [(0) for _ in range(self.width * self.height)]
        def contrast(self, value): # brightness in luma is actually contrast
            self.brightness = value
        def brightness(self, value): # brightness in luma is actually contrast
            self.contrast = value
    class MockDraw:
        def __init__(self, device):
            self.device = device
        def text(self, xy, text, fill=1, font=None):
            self.device.logger.debug(f"MockDraw: Text '{text}' at {xy}")
        def point(self, xy, fill=1):
            self.device.logger.debug(f"MockDraw: Point {xy} fill {fill}")
        def bitmap(self, xy, image, fill=1):
            self.device.logger.debug(f"MockDraw: Bitmap at {xy}")


class LEDMatrixNode(Node):
    """
    ROS2 Node for controlling cascaded MAX7219 LED Matrix 8x8 displays.
    Supports displaying text, numbers, or custom bitmaps.
    """

    def __init__(self):
        super().__init__('led_matrix_node')

        # 1. Declare Parameters
        self.declare_parameter('cascaded_devices', 1)    # Number of cascaded MAX7219 devices.
        self.declare_parameter('gpio_port', 0)           # SPI Port (0 or 1).
        self.declare_parameter('gpio_device', 0)         # SPI Device (CS0 or CS1).
        self.declare_parameter('block_orientation', 0)   # 0, 90, -90, or 180 degrees.
        self.declare_parameter('rotate', 0)              # Rotation for each device (0, 1, 2, 3).
        self.declare_parameter('brightness', 50)         # Brightness (1 to 255).
        self.declare_parameter('mock_hardware', False)   # For testing without real hardware.

        # 2. Read Parameters
        self.cascaded_devices = self.get_parameter('cascaded_devices').value
        self.gpio_port = self.get_parameter('gpio_port').value
        self.gpio_device = self.get_parameter('gpio_device').value
        self.block_orientation = self.get_parameter('block_orientation').value
        self.rotate = self.get_parameter('rotate').value
        self.brightness = self.get_parameter('brightness').value
        self.mock_mode = self.get_parameter('mock_hardware').value

        self.device = None
        if not self.mock_mode:
            try:
                # Setup SPI interface
                # You might need to adjust bus_speed_hz based on your setup.
                # Default bus_speed_hz=8000000 (8MHz) is usually fine.
                serial_interface = spi(port=self.gpio_port, device=self.gpio_device, cs_high=False, bus_speed_hz=8000000)
                
                # Setup MAX7219 device
                self.device = max7219(
                    serial_interface,
                    cascaded=self.cascaded_devices,
                    block_orientation=self.block_orientation,
                    rotate=self.rotate,
                    brightness=self.brightness
                )
                self.device.clear() # Clear display
                self.get_logger().info(f'LEDMatrix: Initialized with {self.cascaded_devices} cascaded devices on SPI port {self.gpio_port}, device {self.gpio_device}.')
            except Exception as e:
                self.get_logger().error(f'LEDMatrix: Failed to initialize luma.led_matrix: {e}. '
                                        'Ensure SPI is enabled and `luma.led_matrix` is installed. '
                                        'Falling back to mock mode.')
                self.mock_mode = True
        
        if self.mock_mode:
            self.get_logger().warn('LEDMatrix: Running in MOCK mode. No real LED matrix will be controlled.')
            self.device = max7219(noop(), cascaded=self.cascaded_devices, block_orientation=self.block_orientation, rotate=self.rotate, brightness=self.brightness)
        
        self.current_text = ""
        self.current_bitmap = None # For Int8MultiArray

        # 3. Subscribers
        qos_profile = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)

        # For displaying text
        self.text_sub = self.create_subscription(
            String,
            '~/display_text',
            self.display_text_callback,
            qos_profile
        )
        # For displaying a custom bitmap (e.g., 8x8 or 8x(N*8) for cascaded)
        # Int8MultiArray where data is 0 for off, 1 for on, row by row.
        # Length should be device.width * device.height / 8 for a full image
        self.bitmap_sub = self.create_subscription(
            Int8MultiArray,
            '~/display_bitmap',
            self.display_bitmap_callback,
            qos_profile
        )
        self.get_logger().info(f'LEDMatrix: Subscribing to display commands.')

        # Initialize with empty/clear display
        self.device.clear()
        self.device.show()


    def display_text_callback(self, msg: String):
        """Displays text on the LED matrix."""
        self.current_text = msg.data
        self.device.clear()
        with canvas(self.device) as draw:
            # luma.led_matrix handles fonts and rendering
            # Default font for luma is tiny, we could specify a custom font
            # For simplicity, just render at (0,0)
            draw.text((0, 0), self.current_text, fill="white")
        self.device.show()
        self.get_logger().debug(f"LEDMatrix: Displayed text: '{self.current_text}'")

    def display_bitmap_callback(self, msg: Int8MultiArray):
        """Displays a custom bitmap on the LED matrix."""
        # Expecting a flat array of 0s and 1s corresponding to pixels
        # E.g., for an 8x8 matrix, an array of 64 elements
        expected_len = self.device.width * self.device.height
        if len(msg.data) != expected_len:
            self.get_logger().warn(f"LEDMatrix: Received bitmap with {len(msg.data)} elements, expected {expected_len}. Ignoring.")
            return

        self.device.clear()
        for y in range(self.device.height):
            for x in range(self.device.width):
                pixel_index = y * self.device.width + x
                value = msg.data[pixel_index]
                self.device.set_pixel(x, y, value) # Value 0 or 1
        self.device.show()
        self.get_logger().debug(f"LEDMatrix: Displayed custom bitmap.")


    def destroy_node(self):
        if self.device:
            self.device.clear() # Clear display on shutdown
            self.device.show()
            self.get_logger().info('LEDMatrix: Display cleared.')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LEDMatrixNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
