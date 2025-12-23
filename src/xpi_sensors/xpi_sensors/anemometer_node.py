#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from gpiozero import Button, Device
from gpiozero.pins.mock import MockFactory
import os
import time

class AnemometerNode(Node):
    """
    ROS2 Node for a mechanical Anemometer (wind speed sensor).
    Counts pulses from a reed switch or Hall effect sensor over a sliding window.
    """

    def __init__(self):
        super().__init__('anemometer_node')

        # 1. Parameters
        self.declare_parameter('gpio_pin', 5) # BCM Pin
        self.declare_parameter('update_interval', 5.0) # Seconds to calculate average
        self.declare_parameter('factor', 0.666) # Conversion factor: 1 Hz = X m/s (0.666 m/s = 2.4 km/h)
        self.declare_parameter('frame_id', 'anemometer_link')
        self.declare_parameter('mock_hardware', False)

        pin_num = self.get_parameter('gpio_pin').value
        self.interval = self.get_parameter('update_interval').value
        self.factor = self.get_parameter('factor').value
        self.frame_id = self.get_parameter('frame_id').value
        mock_mode = self.get_parameter('mock_hardware').value

        # 2. Mock Hardware Setup
        if mock_mode or os.environ.get('GPIOZERO_PIN_FACTORY') == 'mock':
            self.get_logger().warn('Running in MOCK mode.')
            Device.pin_factory = MockFactory()

        # 3. Hardware Init (using Button as it handles debounce and interrupts well)
        try:
            # pull_up=True assumes the switch connects the pin to Ground
            self.button = Button(pin_num, pull_up=True, bounce_time=0.01)
            self.button.when_pressed = self._pulse_callback
            self.get_logger().info(f'Anemometer initialized on GPIO{pin_num}')
        except Exception as e:
            self.get_logger().error(f'Failed to init GPIO{pin_num}: {e}')
            self.button = None

        self.pulse_count = 0
        self.last_time = time.time()

        # 4. Publisher & Timer
        self.pub = self.create_publisher(Float32, '~/wind_speed', 10)
        self.timer = self.create_timer(self.interval, self.timer_callback)

    def _pulse_callback(self):
        """Called on every pulse (revolution)"""
        self.pulse_count += 1

    def timer_callback(self):
        """Calculate and publish wind speed based on pulses over interval"""
        current_time = time.time()
        elapsed = current_time - self.last_time
        
        if elapsed <= 0:
            return

        # Calculate Frequency (Pulses per second)
        freq = self.pulse_count / elapsed
        
        # Wind Speed = Freq * Factor
        wind_speed = freq * self.factor

        # Reset for next interval
        self.pulse_count = 0
        self.last_time = current_time

        # Publish
        msg = Float32()
        msg.data = float(wind_speed)
        self.pub.publish(msg)

        self.get_logger().debug(f'Wind Speed: {wind_speed:.2f} m/s (Freq: {freq:.2f} Hz)')

    def destroy_node(self):
        if hasattr(self, 'button') and self.button:
            self.button.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AnemometerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
