import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from xpi_commons.i2c_helper import get_smbus, MockSMBus
import os
import time

try:
    from luma.core.interface.serial import i2c
    from luma.core.render import canvas
    from luma.oled.device import ssd1306, ssd1309, ssd1325, ssd1331, ssd1351, sh1106
    from PIL import ImageDraw, ImageFont, Image
    
    # Mapping string resolution to luma.oled device classes
    OLED_RESOLUTIONS = {
        "128x64": ssd1306,
        "128x32": ssd1306, # Also handled by ssd1306
        "96x16": ssd1306,
        "128x128": ssd1309, # Though ssd1306 can sometimes handle 128x128 too
        "64x48": sh1106,
    }

except ImportError:
    class i2c:
        def __init__(self, port=0, address=0, bus=None, **kwargs):
            pass
    class canvas:
        def __init__(self, device):
            self.device = device
        def __enter__(self):
            return MockDraw(self.device)
        def __exit__(self, exc_type, exc_val, exc_tb):
            pass
    class MockOLEDDevice:
        def __init__(self, serial_interface, width, height, rotate=0, **kwargs):
            self.width = width
            self.height = height
            self.rotate = rotate
            self.logger = logging.getLogger('MockOLED')
            self.logger.info(f"MockOLED initialized. Resolution: {width}x{height}")
            self.screen_buffer = [" " * width for _ in range(height)]
        def clear(self):
            self.screen_buffer = [" " * self.width for _ in range(self.height)]
        def display(self, image):
            # In mock mode, just log what would be displayed
            # self.logger.info("MockOLED display updated.")
            pass
    class ssd1306(MockOLEDDevice):
        pass
    class ssd1309(MockOLEDDevice):
        pass
    class ImageDraw:
        def __init__(self, image):
            pass
        def text(self, xy, text, fill=None, font=None):
            pass
        def point(self, xy, fill=None):
            pass
    class ImageFont:
        @staticmethod
        def load_default():
            return None
        @staticmethod
        def truetype(font_file, size):
            return None
    class Image:
        @staticmethod
        def new(mode, size):
            return None
    OLED_RESOLUTIONS = {
        "128x64": ssd1306,
        "128x32": ssd1306,
    }


class SSD1306Node(Node):
    """
    ROS2 Node for controlling SSD1306-based I2C OLED displays.
    Supports displaying text and simple bitmaps.
    """

    def __init__(self):
        super().__init__('ssd1306_node')

        # 1. Declare Parameters
        self.declare_parameter('i2c_bus', 1)             # I2C Bus Number.
        self.declare_parameter('i2c_address', 0x3C)      # OLED I2C address (0x3C or 0x3D).
        self.declare_parameter('width', 128)             # Display width (e.g., 128).
        self.declare_parameter('height', 64)             # Display height (e.g., 64 or 32).
        self.declare_parameter('rotation', 0)            # Display rotation (0, 90, 180, 270).
        self.declare_parameter('mock_hardware', False)   # For testing without real hardware.
        self.declare_parameter('font_size', 10)          # Font size for text display.
        self.declare_parameter('default_text', 'Waiting...') # Text to display on start.


        # 2. Read Parameters
        self.i2c_bus = self.get_parameter('i2c_bus').value
        self.i2c_address = self.get_parameter('i2c_address').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.rotation = self.get_parameter('rotation').value
        self.mock_mode = self.get_parameter('mock_hardware').value
        self.font_size = self.get_parameter('font_size').value
        self.default_text = self.get_parameter('default_text').value

        self.oled_device = None
        if not self.mock_mode:
            try:
                # Use xpi_commons I2C helper to get mockable bus
                i2c_bus_obj = get_smbus(self.i2c_bus)
                
                # Check for specific device resolution and type
                resolution_key = f"{self.width}x{self.height}"
                oled_class = OLED_RESOLUTIONS.get(resolution_key, ssd1306) # Default to ssd1306

                # Setup I2C interface for luma
                # luma's i2c interface doesn't directly take smbus2 object, so we mock its behavior for real i2c too
                serial_interface = i2c(port=self.i2c_bus, address=self.i2c_address, bus=i2c_bus_obj)
                
                # Setup OLED device
                self.oled_device = oled_class(
                    serial_interface,
                    width=self.width,
                    height=self.height,
                    rotate=self.rotation
                )
                self.oled_device.clear() # Clear display
                self.get_logger().info(f'OLED: Initialized {resolution_key} display at 0x{self.i2c_address:02X} on bus {self.i2c_bus}.')
            except Exception as e:
                self.get_logger().error(f'OLED: Failed to initialize luma.oled: {e}. '
                                        'Ensure I2C is enabled and `luma.oled` is installed. '
                                        'Falling back to mock mode.')
                self.mock_mode = True
        
        if self.mock_mode:
            self.get_logger().warn('OLED: Running in MOCK mode. No real OLED will be controlled.')
            self.oled_device = ssd1306(i2c(port=self.i2c_bus, address=self.i2c_address, bus=MockSMBus()), 
                                       width=self.width, height=self.height, rotate=self.rotation)

        try:
            # Load default font
            self.font = ImageFont.truetype("arial.ttf", self.font_size)
        except IOError:
            self.get_logger().warn("Could not load 'arial.ttf', using default PIL font.")
            self.font = ImageFont.load_default()

        # Display default text
        self.display_text_on_oled(self.default_text)

        # 3. Subscribers
        qos_profile = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)

        # For displaying text
        self.text_sub = self.create_subscription(
            String,
            '~/display_text',
            self.display_text_callback,
            qos_profile
        )
        # For displaying a custom bitmap (raw 1-bit image data)
        # Int8MultiArray where data is 0 for off, 1 for on, column by column or row by row.
        # Length should be device.width * device.height
        self.bitmap_sub = self.create_subscription(
            Int8MultiArray,
            '~/display_bitmap',
            self.display_bitmap_callback,
            qos_profile
        )
        self.get_logger().info(f'OLED: Subscribing to display commands.')


    def display_text_on_oled(self, text_to_display: str):
        """Helper to render text on the OLED."""
        if not self.oled_device: return
        self.oled_device.clear()
        with canvas(self.oled_device) as draw:
            draw.text((0, 0), text_to_display, fill="white", font=self.font)
        self.oled_device.display() # Luma uses display() not show()
        self.get_logger().debug(f"OLED: Displayed text: '{text_to_display}'")

    def display_text_callback(self, msg: String):
        """Displays text on the OLED screen."""
        self.display_text_on_oled(msg.data)
    
    def display_bitmap_callback(self, msg: Int8MultiArray):
        """Displays a custom bitmap on the OLED screen."""
        if not self.oled_device: return
        
        expected_len = self.width * self.height
        if len(msg.data) != expected_len:
            self.get_logger().warn(f"OLED: Received bitmap with {len(msg.data)} elements, expected {expected_len}. Ignoring.")
            return

        self.oled_device.clear()
        # Create a PIL Image from the bitmap data
        image = Image.new("1", (self.width, self.height)) # "1" for 1-bit pixels (black and white)
        pixels = image.load()
        for y in range(self.height):
            for x in range(self.width):
                pixels[x, y] = msg.data[y * self.width + x] * 255 # 0 or 255 for black/white

        self.oled_device.display(image)
        self.get_logger().debug(f"OLED: Displayed custom bitmap.")


    def destroy_node(self):
        if self.oled_device:
            self.oled_device.clear() # Clear display on shutdown
            self.oled_device.display()
            self.get_logger().info('OLED: Display cleared.')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SSD1306Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
