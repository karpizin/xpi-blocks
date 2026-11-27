import math
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from xpi_inputs.ppm_decoder import PpmDecoder

try:
    import pigpio
except ImportError:  # mock mode still works without it
    pigpio = None


class PpmReceiverNode(Node):
    """
    ROS2 Node for receiving PPM (CPPM) data on a single GPIO pin via pigpio.
    Publishes decoded channel values (-1.0..1.0) as sensor_msgs/Joy on ~/joy.

    pigpio timestamps every edge in its daemon with microsecond resolution, which is what
    PPM decoding needs (Python-level edge timing is too jittery). Requires the daemon:
    ``sudo pigpiod`` (or ``sudo systemctl enable --now pigpiod``). pigpio does not support
    the Raspberry Pi 5.

    When no complete frame arrives for signal_timeout seconds (receiver off, out of range,
    cable unplugged) the node stops publishing, so downstream command watchdogs stop the robot.
    """

    def __init__(self):
        super().__init__('ppm_receiver_node')

        # 1. Declare Parameters
        self.declare_parameter('gpio_pin', 17)  # BCM pin for the PPM input
        self.declare_parameter('publish_rate', 50.0)  # Hz
        self.declare_parameter('mock_hardware', False)  # For testing without real GPIO

        # PPM specific parameters
        self.declare_parameter('num_channels', 8)
        self.declare_parameter('frame_timeout_us', 3000)  # sync gap: an interval this long starts a frame
        self.declare_parameter('min_pulse_us', 900)   # shortest valid channel interval
        self.declare_parameter('max_pulse_us', 2100)  # longest valid channel interval
        self.declare_parameter('center_pulse_us', 1500)  # interval that maps to 0.0
        self.declare_parameter('edge', 'rising')  # edge that starts each pulse: 'rising' or 'falling'
        self.declare_parameter('pull_up', True)  # enable the internal pull-up on the pin
        self.declare_parameter('signal_timeout', 0.25)  # s without a frame = signal lost

        # 2. Read Parameters
        self.gpio_pin = self.get_parameter('gpio_pin').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.mock_mode = self.get_parameter('mock_hardware').value
        self.num_channels = self.get_parameter('num_channels').value
        self.signal_timeout = float(self.get_parameter('signal_timeout').value)
        edge = str(self.get_parameter('edge').value).lower()

        self.decoder = PpmDecoder(
            num_channels=self.num_channels,
            sync_gap_us=self.get_parameter('frame_timeout_us').value,
            min_pulse_us=self.get_parameter('min_pulse_us').value,
            max_pulse_us=self.get_parameter('max_pulse_us').value,
            center_pulse_us=self.get_parameter('center_pulse_us').value,
        )
        self._lock = threading.Lock()   # pigpio calls _on_edge from its own thread
        self.decoded_channels = None    # last complete frame
        self.last_frame_rx_time = 0.0
        self.signal_ok = False
        self.pi = None
        self._edge_cb = None

        # 3. Hardware: pigpio edge callback
        if self.mock_mode:
            self.get_logger().warn('PPM: Running in MOCK mode. No real GPIO will be used.')
        elif edge not in ('rising', 'falling'):
            self.get_logger().error(f"PPM: edge must be 'rising' or 'falling', got '{edge}'.")
        else:
            self._start_pigpio(edge, bool(self.get_parameter('pull_up').value))

        # 4. Publisher and timer
        self.joy_publisher = self.create_publisher(Joy, '~/joy', 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        self.mock_channel_val = 0.0  # For mock data simulation

    def _start_pigpio(self, edge, pull_up):
        if pigpio is None:
            self.get_logger().error("PPM: the 'pigpio' Python package is not installed (pip install pigpio).")
            return
        pi = pigpio.pi()
        if not pi.connected:
            self.get_logger().error('PPM: cannot connect to the pigpio daemon; start it with `sudo pigpiod`.')
            return
        self.pi = pi
        pi.set_mode(self.gpio_pin, pigpio.INPUT)
        pi.set_pull_up_down(self.gpio_pin, pigpio.PUD_UP if pull_up else pigpio.PUD_OFF)
        level = pigpio.RISING_EDGE if edge == 'rising' else pigpio.FALLING_EDGE
        self._edge_cb = pi.callback(self.gpio_pin, level, self._on_edge)
        self.get_logger().info(f'PPM: listening on GPIO {self.gpio_pin} ({edge} edges, pigpio).')

    def _on_edge(self, gpio, level, tick):
        """pigpio callback (pigpio thread): tick is the edge time in microseconds."""
        if level == pigpio.TIMEOUT:
            return
        with self._lock:
            frame = self.decoder.feed(tick)
            if frame is not None:
                self.decoded_channels = frame
                self.last_frame_rx_time = time.monotonic()

    def timer_callback(self):
        if self.mock_mode:
            self.publish_mock_data()
            return

        with self._lock:
            channels = self.decoded_channels
            age = time.monotonic() - self.last_frame_rx_time

        fresh = channels is not None and age < self.signal_timeout
        if fresh != self.signal_ok:
            self.signal_ok = fresh
            if fresh:
                self.get_logger().info('PPM: signal acquired.')
            elif channels is not None:
                self.get_logger().warn(f'PPM: no frame for {self.signal_timeout:.2f} s, signal lost.')
        if not fresh:
            return

        joy_msg = Joy()
        joy_msg.header.stamp = self.get_clock().now().to_msg()
        joy_msg.axes = [float(v) for v in channels]
        joy_msg.buttons = []  # PPM carries only channel values
        self.joy_publisher.publish(joy_msg)

    def publish_mock_data(self):
        """ Publishes dummy Joy messages for mock mode. """
        current_time = time.monotonic()
        joy_msg = Joy()
        joy_msg.header.stamp = self.get_clock().now().to_msg()
        # Simulate moving stick
        self.mock_channel_val = math.sin(current_time * math.pi / 2.0)  # Sine wave between -1 and 1
        joy_msg.axes = [
            self.mock_channel_val,  # Channel 0
            -self.mock_channel_val,  # Channel 1
        ] + [0.0] * (self.num_channels - 2)  # Fill rest with 0s
        joy_msg.buttons = []
        self.joy_publisher.publish(joy_msg)

    def destroy_node(self):
        if self._edge_cb is not None:
            self._edge_cb.cancel()
        if self.pi is not None:
            self.pi.stop()
            self.get_logger().info('PPM: pigpio connection closed.')
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
