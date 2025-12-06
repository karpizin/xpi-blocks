import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from gpiozero import PulseInput, Device
from gpiozero.pins.mock import MockFactory
import os
import math
import time

class PpmReceiverNode(Node):
    """
    ROS2 Node for receiving PPM (Pulse Position Modulation) data via a single GPIO pin.
    Publishes decoded channel values as sensor_msgs/Joy.
    """

    def __init__(self):
        super().__init__('ppm_receiver_node')

        # 1. Declare Parameters
        self.declare_parameter('gpio_pin', 17) # Default GPIO pin for PPM input
        self.declare_parameter('publish_rate', 50.0) # Hz
        self.declare_parameter('mock_hardware', False) # For testing without real GPIO
        
        # PPM specific parameters
        self.declare_parameter('num_channels', 8)
        self.declare_parameter('frame_timeout_us', 5000) # Microseconds, time between frames
        self.declare_parameter('min_pulse_us', 900)   # Microseconds, typical min pulse width
        self.declare_parameter('max_pulse_us', 2100)  # Microseconds, typical max pulse width
        self.declare_parameter('center_pulse_us', 1500) # Microseconds, typical center pulse width

        # 2. Read Parameters
        self.gpio_pin = self.get_parameter('gpio_pin').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.mock_mode = self.get_parameter('mock_hardware').value
        self.num_channels = self.get_parameter('num_channels').value
        self.frame_timeout_us = self.get_parameter('frame_timeout_us').value
        self.min_pulse_us = self.get_parameter('min_pulse_us').value
        self.max_pulse_us = self.get_parameter('max_pulse_us').value
        self.center_pulse_us = self.get_parameter('center_pulse_us').value

        # Initialize mock factory if needed
        if self.mock_mode or os.environ.get('GPIOZERO_PIN_FACTORY') == 'mock':
            self.get_logger().warn('PPM: Running in MOCK mode. No real GPIO will be used.')
            Device.pin_factory = MockFactory()

        self.pulse_input = None
        self.last_pulse_start = 0.0 # seconds
        self.pulse_widths = [] # List of pulse widths in us

        try:
            self.pulse_input = PulseInput(self.gpio_pin, pull_up=True) # PPM usually requires pull-up
            self.pulse_input.when_activated = self._pulse_start
            self.pulse_input.when_deactivated = self._pulse_end
            self.get_logger().info(f'PPM: Initialized PulseInput on GPIO {self.gpio_pin}.')
        except Exception as e:
            self.get_logger().error(f'PPM: Failed to initialize PulseInput on GPIO {self.gpio_pin}: {e}')
            self.mock_mode = True # Fallback to mock if real port fails

        # 3. Publisher
        self.joy_publisher = self.create_publisher(Joy, '~/joy', 10)
        
        # 4. Timer for publishing (and mock data)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.decoded_channels = [0.0] * self.num_channels # Normalized values -1.0 to 1.0
        self.last_frame_rx_time = time.monotonic()
        self.mock_channel_val = 0.0 # For mock data simulation

    def _pulse_start(self):
        self.last_pulse_start = time.monotonic()

    def _pulse_end(self):
        if self.last_pulse_start == 0.0:
            return # Ignore if we missed the start
        
        duration = (time.monotonic() - self.last_pulse_start) * 1_000_000 # in microseconds
        
        # Check for frame timeout (gap between frames)
        if duration > self.frame_timeout_us and len(self.pulse_widths) > 0:
            self.process_ppm_frame()
            self.pulse_widths = [] # Reset for new frame
        
        # If it's a channel pulse
        if self.min_pulse_us <= duration <= self.max_pulse_us:
            self.pulse_widths.append(duration)
        # else:
            # self.get_logger().debug(f"PPM: Ignored pulse width {duration:.1f}us")

    def process_ppm_frame(self):
        """
        Process the collected pulse widths as a full PPM frame.
        """
        if not self.pulse_widths:
            return

        if len(self.pulse_widths) < self.num_channels:
            self.get_logger().warn(f"PPM: Incomplete frame (expected {self.num_channels}, got {len(self.pulse_widths)}). Skipping.")
            return

        channels_raw = self.pulse_widths[:self.num_channels]
        
        # Normalize raw pulse widths to -1.0 to 1.0 range
        norm_channels = []
        for pulse_us in channels_raw:
            # Scale from [min_pulse_us, max_pulse_us] to [-1.0, 1.0]
            norm_val = (pulse_us - self.center_pulse_us) / \
                       ((self.max_pulse_us - self.min_pulse_us) / 2.0)
            norm_channels.append(max(-1.0, min(1.0, norm_val))) # Clamp to valid range
        
        self.decoded_channels = norm_channels
        self.last_frame_rx_time = time.monotonic()

    def timer_callback(self):
        if self.mock_mode:
            self.publish_mock_data()
            return

        # Publish the most recently decoded frame
        if (time.monotonic() - self.last_frame_rx_time) < (1.0 / self.publish_rate) and \
           self.decoded_channels: # Only publish if we have fresh data
            joy_msg = Joy()
            joy_msg.header.stamp = self.get_clock().now().to_msg()
            joy_msg.axes = self.decoded_channels
            joy_msg.buttons = [] # PPM doesn't typically transmit button data directly

            self.joy_publisher.publish(joy_msg)
        else:
            self.get_logger().debug("PPM: No new frame to publish or old data.")

    def publish_mock_data(self):
        """ Publishes dummy Joy messages for mock mode. """
        current_time = time.monotonic()
        if (current_time - self.last_frame_rx_time) < (1.0 / self.publish_rate):
            return # Don't publish too fast
            
        joy_msg = Joy()
        joy_msg.header.stamp = self.get_clock().now().to_msg()
        # Simulate moving stick
        self.mock_channel_val = math.sin(current_time * math.pi / 2.0) # Sine wave between -1 and 1
        joy_msg.axes = [
            self.mock_channel_val,  # Channel 0
            -self.mock_channel_val,  # Channel 1
        ] + [0.0] * (self.num_channels - 2) # Fill rest with 0s
        joy_msg.buttons = []
        self.joy_publisher.publish(joy_msg)
        self.last_frame_rx_time = current_time


    def destroy_node(self):
        if self.pulse_input:
            self.pulse_input.close()
            self.get_logger().info('PPM: Closed PulseInput.')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PpmReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
