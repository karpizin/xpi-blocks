import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from gpiozero import Button, Device
from gpiozero.pins.mock import MockFactory
import time

class RainGaugeNode(Node):
    """
    ROS2 Node for a Tipping Bucket Rain Gauge.
    Counts GPIO pulses and converts them to millimeters of rainfall.
    """

    def __init__(self):
        super().__init__('rain_gauge_node')

        # Parameters
        self.declare_parameter('gpio_pin', 22)
        self.declare_parameter('mm_per_pulse', 0.2794) # Standard for Misol/Fine Offset
        self.declare_parameter('publish_rate', 1.0) # Hz
        self.declare_parameter('use_mock', False)

        pin_num = self.get_parameter('gpio_pin').value
        self.mm_per_pulse = self.get_parameter('mm_per_pulse').value
        publish_rate = self.get_parameter('publish_rate').value
        use_mock = self.get_parameter('use_mock').value

        if use_mock:
            Device.pin_factory = MockFactory()
            self.get_logger().info("Using Mock GPIO for Rain Gauge")

        # State
        self.total_mm = 0.0
        self.last_pulse_time = time.monotonic()
        self.pulse_intervals = [] # To calculate intensity

        # GPIO Setup
        # Pull-up is enabled by default in Button
        self.sensor = Button(pin_num, bounce_time=0.05) 
        self.sensor.when_pressed = self.pulse_callback

        # Publishers
        self.total_pub = self.create_publisher(Float32, '~/total', 10)
        self.intensity_pub = self.create_publisher(Float32, '~/intensity', 10)

        # Timer
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)
        
        self.get_logger().info(f"Rain Gauge initialized on GPIO {pin_num}")

    def pulse_callback(self):
        now = time.monotonic()
        self.total_mm += self.mm_per_pulse
        
        interval = now - self.last_pulse_time
        self.pulse_intervals.append(interval)
        
        # Keep only last 10 intervals for rolling average
        if len(self.pulse_intervals) > 10:
            self.pulse_intervals.pop(0)
            
        self.last_pulse_time = now
        self.get_logger().debug(f"Rain Pulse! Total: {self.total_mm:.4f} mm")

    def timer_callback(self):
        # Calculate Intensity (mm/hour)
        intensity = 0.0
        now = time.monotonic()
        
        # If no pulses for 5 minutes, intensity is 0
        if now - self.last_pulse_time > 300:
            intensity = 0.0
        elif len(self.pulse_intervals) > 0:
            avg_interval = sum(self.pulse_intervals) / len(self.pulse_intervals)
            # intensity = (mm/pulse) / (seconds/pulse) * 3600 seconds/hour
            intensity = (self.mm_per_pulse / avg_interval) * 3600.0

        # Publish
        msg_total = Float32()
        msg_total.data = float(self.total_mm)
        self.total_pub.publish(msg_total)

        msg_intensity = Float32()
        msg_intensity.data = float(intensity)
        self.intensity_pub.publish(msg_intensity)

def main(args=None):
    rclpy.init(args=args)
    node = RainGaugeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sensor.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
