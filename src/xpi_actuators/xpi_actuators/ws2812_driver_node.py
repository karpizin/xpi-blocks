import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time
import os
import math

try:
    from rpi_ws281x import PixelStrip, Color
    # Optional: Mock Color and PixelStrip for non-RPi environments
except ImportError:
    class Color:
        def __init__(self, r, g, b, w=0):
            pass
    class PixelStrip:
        def __init__(self, num, pin, freq=800000, dma=10, invert=False, brightness=255, channel=0, strip_type=None):
            pass
        def begin(self):
            pass
        def setPixelColor(self, pixel, color):
            pass
        def show(self):
            pass
        def numPixels(self):
            return 0
        def getPixelColor(self, pixel):
            return Color(0,0,0)


class WS2812DriverNode(Node):
    """
    ROS2 Node for controlling WS2812B/NeoPixel addressable LEDs.
    Subscribes to ColorRGBA messages to set individual pixel colors.
    """

    def __init__(self):
        super().__init__('ws2812_driver_node')

        # 1. Declare Parameters
        self.declare_parameter('led_count', 30)          # Number of LED pixels.
        self.declare_parameter('led_pin', 18)            # GPIO pin connected to the pixels (18 uses PWM, 10 uses PCM).
        self.declare_parameter('led_freq_hz', 800000)    # LED signal frequency in hertz (usually 800khz).
        self.declare_parameter('led_dma', 10)            # DMA channel to use for generating signal (try 10).
        self.declare_parameter('led_brightness', 255)    # Set to 0 for darkest and 255 for brightest.
        self.declare_parameter('led_invert', False)      # True to invert the signal (when using NPN transistor level shift).
        self.declare_parameter('led_channel', 0)         # Set to '1' for GPIOs 13, 19, 40, 52.
        self.declare_parameter('mock_hardware', False)   # For testing without real WS2812
        self.declare_parameter('update_rate', 30.0)      # Max Hz for publishing updates to LEDs

        # 2. Read Parameters
        self.led_count = self.get_parameter('led_count').value
        self.led_pin = self.get_parameter('led_pin').value
        self.led_freq_hz = self.get_parameter('led_freq_hz').value
        self.led_dma = self.get_parameter('led_dma').value
        self.led_brightness = self.get_parameter('led_brightness').value
        self.led_invert = self.get_parameter('led_invert').value
        self.led_channel = self.get_parameter('led_channel').value
        self.mock_mode = self.get_parameter('mock_hardware').value
        self.update_rate = self.get_parameter('update_rate').value

        self.strip = None
        self.pixel_colors = [Color(0,0,0)] * self.led_count # Store current pixel states
        
        if not self.mock_mode:
            try:
                # Create NeoPixel object with appropriate configuration.
                self.strip = PixelStrip(
                    self.led_count,
                    self.led_pin,
                    self.led_freq_hz,
                    self.led_dma,
                    self.led_invert,
                    self.led_brightness,
                    self.led_channel
                )
                # Intialize the library (must be called once before other functions).
                self.strip.begin()
                self.get_logger().info(f'WS2812: Initialized with {self.led_count} LEDs on GPIO {self.led_pin}.')
                self.set_all_pixels(Color(0,0,0)) # Turn off all LEDs initially
            except Exception as e:
                self.get_logger().error(f'WS2812: Failed to initialize rpi_ws281x: {e}. '
                                        'Ensure you have `rpi_ws281x` installed and running with sudo/correct permissions. '
                                        'Falling back to mock mode.')
                self.mock_mode = True
        
        if self.mock_mode:
            self.get_logger().warn('WS2812: Running in MOCK mode. No real LEDs will be controlled.')
            self.mock_pattern_step = 0
            self.last_mock_update = time.monotonic()


        # 3. Subscribers (for individual pixel control)
        qos_profile = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)

        self.pixel_subscriptions = {}
        for i in range(self.led_count):
            self.pixel_subscriptions[i] = self.create_subscription(
                ColorRGBA,
                f'~/pixel_{i}/set_color',
                lambda msg, pixel_id=i: self._set_pixel_callback(pixel_id, msg),
                qos_profile
            )
        self.get_logger().info(f'WS2812: Subscribing to individual pixel color topics.')
        
        # Subscriber for whole strip
        self.strip_subscription = self.create_subscription(
            ColorRGBA, # Single ColorRGBA to set all pixels to
            '~/strip/set_color',
            self._set_strip_color_callback,
            qos_profile
        )
        self.get_logger().info(f'WS2812: Subscribing to strip color topic.')

        # Timer to periodically call show() or mock updates
        self.timer = self.create_timer(1.0 / self.update_rate, self.timer_callback)


    def _set_pixel_callback(self, pixel_id: int, msg: ColorRGBA):
        """Callback to set the color of a specific pixel."""
        if 0 <= pixel_id < self.led_count:
            if not self.mock_mode and self.strip:
                color = Color(int(msg.r*255), int(msg.g*255), int(msg.b*255), int(msg.a*255))
                self.strip.setPixelColor(pixel_id, color)
                # self.strip.show() # Call show() in timer_callback for efficiency
                self.pixel_colors[pixel_id] = color
                self.get_logger().debug(f"Set pixel {pixel_id} to R:{msg.r*255} G:{msg.g*255} B:{msg.b*255}")
            else:
                self.get_logger().debug(f"Mock: Set pixel {pixel_id} to R:{msg.r*255} G:{msg.g*255} B:{msg.b*255}")
        else:
            self.get_logger().warn(f"Invalid pixel ID: {pixel_id}. Must be between 0 and {self.led_count-1}.")

    def _set_strip_color_callback(self, msg: ColorRGBA):
        """Callback to set all pixels to a single color."""
        color = Color(int(msg.r*255), int(msg.g*255), int(msg.b*255), int(msg.a*255))
        if not self.mock_mode and self.strip:
            for i in range(self.led_count):
                self.strip.setPixelColor(i, color)
            # self.strip.show() # Call show() in timer_callback for efficiency
            self.pixel_colors = [color] * self.led_count
            self.get_logger().info(f"Set all pixels to R:{msg.r*255} G:{msg.g*255} B:{msg.b*255}")
        else:
            self.get_logger().info(f"Mock: Set all pixels to R:{msg.r*255} G:{msg.g*255} B:{msg.b*255}")
            self.pixel_colors = [color] * self.led_count


    def set_all_pixels(self, color):
        if not self.mock_mode and self.strip:
            for i in range(self.strip.numPixels()):
                self.strip.setPixelColor(i, color)
            self.strip.show()
        self.pixel_colors = [color] * self.led_count

    def timer_callback(self):
        """Periodically update the LED strip (or simulate in mock mode)."""
        if self.mock_mode:
            # Simulate a simple chasing pattern in mock mode
            current_time = time.monotonic()
            if (current_time - self.last_mock_update) > (1.0 / self.update_rate):
                self.mock_pattern_step = (self.mock_pattern_step + 1) % self.led_count
                # self.get_logger().debug(f"Mock: LED {self.mock_pattern_step} active.")
                self.last_mock_update = current_time
        elif self.strip:
            # Only call show if there's been changes
            self.strip.show() # This sends the data to the strip
            
    def destroy_node(self):
        if not self.mock_mode and self.strip:
            self.set_all_pixels(Color(0,0,0)) # Turn off all LEDs on shutdown
            self.get_logger().info('WS2812: Turning off LEDs.')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WS2812DriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
