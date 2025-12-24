import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, Int32MultiArray
from gpiozero import DigitalOutputDevice, DigitalInputDevice, Device
from gpiozero.pins.mock import MockFactory
import time
import threading

class TCS3200Node(Node):
    """
    ROS2 Node for the TCS3200 (GY-31) color sensor.
    Cycles through RGB filters and measures output frequency via GPIO.
    """

    def __init__(self):
        super().__init__('tcs3200_node')

        # Parameters
        self.declare_parameter('s0_pin', 23)
        self.declare_parameter('s1_pin', 24)
        self.declare_parameter('s2_pin', 25)
        self.declare_parameter('s3_pin', 8)
        self.declare_parameter('out_pin', 7)
        self.declare_parameter('led_pin', 12)
        self.declare_parameter('sample_duration', 0.1) # seconds per channel
        self.declare_parameter('publish_rate', 2.0) # Hz
        self.declare_parameter('mock_hardware', False)

        pins = {
            's0': self.get_parameter('s0_pin').value,
            's1': self.get_parameter('s1_pin').value,
            's2': self.get_parameter('s2_pin').value,
            's3': self.get_parameter('s3_pin').value,
            'out': self.get_parameter('out_pin').value,
            'led': self.get_parameter('led_pin').value
        }
        self.sample_duration = self.get_parameter('sample_duration').value
        publish_rate = self.get_parameter('publish_rate').value
        mock_mode = self.get_parameter('mock_hardware').value

        if mock_mode:
            Device.pin_factory = MockFactory()
            self.get_logger().warn('TCS3200: Running in MOCK mode.')

        # GPIO Setup
        self.s0 = DigitalOutputDevice(pins['s0'])
        self.s1 = DigitalOutputDevice(pins['s1'])
        self.s2 = DigitalOutputDevice(pins['s2'])
        self.s3 = DigitalOutputDevice(pins['s3'])
        self.led = DigitalOutputDevice(pins['led'])
        self.out = DigitalInputDevice(pins['out'])

        # Set Frequency Scaling to 20% (S0=L, S1=H) - stable for RPi
        self.s0.off()
        self.s1.on()
        self.led.on() # Turn on illumination

        # Pulse counting state
        self.pulse_count = 0
        self.out.when_activated = self.count_pulse
        
        # Publishers
        self.color_pub = self.create_publisher(ColorRGBA, '~/color', 10)
        self.freq_pub = self.create_publisher(Int32MultiArray, '~/raw_frequencies', 10)

        # Timer for the measuring loop
        self.timer = self.create_timer(1.0 / publish_rate, self.measure_color)
        self.get_logger().info(f"TCS3200 initialized. Sampling {self.sample_duration}s per channel.")

    def count_pulse(self):
        self.pulse_count += 1

    def get_frequency(self, s2_state, s3_state):
        """Measures frequency for a specific filter configuration."""
        self.s2.value = s2_state
        self.s3.value = s3_state
        
        self.pulse_count = 0
        time.sleep(self.sample_duration)
        return self.pulse_count

    def measure_color(self):
        try:
            # S2/S3 Logic:
            # L/L = Red
            # L/H = Blue
            # H/H = Green
            # H/L = Clear
            
            r_freq = self.get_frequency(False, False)
            b_freq = self.get_frequency(False, True)
            g_freq = self.get_frequency(True, True)
            c_freq = self.get_frequency(True, False)

            # Raw Data Msg
            freq_msg = Int32MultiArray()
            freq_msg.data = [r_freq, g_freq, b_freq, c_freq]
            self.freq_pub.publish(freq_msg)

            # Normalized Color (relative to Clear channel)
            color_msg = ColorRGBA()
            if c_freq > 0:
                color_msg.r = min(1.0, float(r_freq) / c_freq)
                color_msg.g = min(1.0, float(g_freq) / c_freq)
                color_msg.b = min(1.0, float(b_freq) / c_freq)
                color_msg.a = 1.0
            
            self.color_pub.publish(color_msg)
            
            self.get_logger().debug(f"RGB: ({color_msg.r:.2f}, {color_msg.g:.2f}, {color_msg.b:.2f})")

        except Exception as e:
            self.get_logger().error(f"TCS3200 error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TCS3200Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.led.off()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
