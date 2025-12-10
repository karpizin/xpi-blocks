import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, String, Float32
from rclpy.parameter import ParameterDescriptor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time
from xpi_actuators.lib.led_effects import LedEffects

try:
    from rpi_ws281x import PixelStrip, Color
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
    ROS2 Node for controlling WS2812B/NeoPixel addressable LEDs with Effect Library support.
    """

    def __init__(self):
        super().__init__('ws2812_driver_node')

        # 1. Hardware Parameters
        self.declare_parameter('led_count', 30, ParameterDescriptor(description='Number of LED pixels'))
        self.declare_parameter('led_pin', 18, ParameterDescriptor(description='GPIO pin (PWM/PCM)'))
        self.declare_parameter('led_freq_hz', 800000)
        self.declare_parameter('led_dma', 10)
        self.declare_parameter('led_brightness', 255)
        self.declare_parameter('led_invert', False)
        self.declare_parameter('led_channel', 0)
        self.declare_parameter('mock_hardware', False)
        self.declare_parameter('update_rate', 30.0)

        # 2. Effect Parameters (Initial State)
        self.declare_parameter('initial_effect', 'solid', ParameterDescriptor(description='Startup effect name'))
        self.declare_parameter('initial_speed', 1.0, ParameterDescriptor(description='Effect speed'))
        self.declare_parameter('initial_color', [0, 0, 0], ParameterDescriptor(description='Initial RGB color [0-255, 0-255, 0-255]'))

        # Read Params
        self.led_count = self.get_parameter('led_count').value
        self.led_pin = self.get_parameter('led_pin').value
        self.led_freq_hz = self.get_parameter('led_freq_hz').value
        self.led_dma = self.get_parameter('led_dma').value
        self.led_brightness = self.get_parameter('led_brightness').value
        self.led_invert = self.get_parameter('led_invert').value
        self.led_channel = self.get_parameter('led_channel').value
        self.mock_mode = self.get_parameter('mock_hardware').value
        self.update_rate = self.get_parameter('update_rate').value

        # Effect State
        self.current_effect = self.get_parameter('initial_effect').value
        self.effect_speed = self.get_parameter('initial_speed').value
        ic = self.get_parameter('initial_color').value
        self.current_color = (ic[0], ic[1], ic[2])

        # Initialize Logic
        self.effects_lib = LedEffects(self.led_count)
        
        # Initialize Hardware
        self.strip = None
        if not self.mock_mode:
            try:
                self.strip = PixelStrip(
                    self.led_count,
                    self.led_pin,
                    self.led_freq_hz,
                    self.led_dma,
                    self.led_invert,
                    self.led_brightness,
                    self.led_channel
                )
                self.strip.begin()
                self.get_logger().info(f'WS2812: Initialized {self.led_count} LEDs on GPIO {self.led_pin}.')
            except Exception as e:
                self.get_logger().error(f'WS2812 Init Failed: {e}. Falling back to mock.')
                self.mock_mode = True
        
        if self.mock_mode:
            self.get_logger().warn('WS2812: Running in MOCK mode.')

        # 3. Interfaces
        # Topics
        self.create_subscription(String, '~/set_effect', self.cb_set_effect, 10)
        self.create_subscription(Float32, '~/set_speed', self.cb_set_speed, 10)
        self.create_subscription(ColorRGBA, '~/set_color', self.cb_set_color, 10)
        
        # Timer
        self.timer = self.create_timer(1.0 / self.update_rate, self.update_loop)
        self.get_logger().info(f'WS2812: Ready. Effect: {self.current_effect}, Speed: {self.effect_speed}')

    def cb_set_effect(self, msg):
        self.current_effect = msg.data
        self.get_logger().info(f"Switched effect to: {self.current_effect}")

    def cb_set_speed(self, msg):
        self.effect_speed = msg.data

    def cb_set_color(self, msg):
        # Setting a solid color implies switching to 'solid' effect or 'blink' etc base color
        # We update the stored color. If the current effect uses it, it will change.
        # If we want to force 'solid' mode on color set:
        self.current_color = (int(msg.r * 255), int(msg.g * 255), int(msg.b * 255))
        # Optional: Force solid effect if user sets color manually?
        # self.current_effect = 'solid' 

    def update_loop(self):
        # 1. Calculate next frame
        pixels = self.effects_lib.update(self.current_effect, self.current_color, self.effect_speed)
        
        # 2. Render
        if not self.mock_mode and self.strip:
            for i, (r, g, b) in enumerate(pixels):
                # Optimize: Check if changed? RPi_WS281x handles it reasonably well.
                # rpi_ws281x expects Color(r,g,b)
                self.strip.setPixelColor(i, Color(r, g, b))
            self.strip.show()
        
    def destroy_node(self):
        if not self.mock_mode and self.strip:
            for i in range(self.strip.numPixels()):
                self.strip.setPixelColor(i, Color(0,0,0))
            self.strip.show()
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