import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from gpiozero import DistanceSensor, Device
from gpiozero.pins.mock import MockFactory
import os

class SonarNode(Node):
    """
    ROS2 Node for HC-SR04 Ultrasonic Sensor using gpiozero.
    Publishes: ~/range (sensor_msgs/Range)
    """

    def __init__(self):
        super().__init__('sonar_node')

        # 1. Parameters
        self.declare_parameter('trigger_pin', 23)
        self.declare_parameter('echo_pin', 24)
        self.declare_parameter('max_distance', 2.0)  # Meters
        self.declare_parameter('frame_id', 'sonar_link')
        self.declare_parameter('update_rate', 10.0)  # Hz
        self.declare_parameter('fov', 0.52) # ~30 degrees in radians
        self.declare_parameter('mock_hardware', False)

        # 2. Config
        trig = self.get_parameter('trigger_pin').value
        echo = self.get_parameter('echo_pin').value
        self.max_dist = self.get_parameter('max_distance').value
        self.frame_id = self.get_parameter('frame_id').value
        rate = self.get_parameter('update_rate').value
        self.fov = self.get_parameter('fov').value
        mock_mode = self.get_parameter('mock_hardware').value

        # 3. Hardware Init
        if mock_mode or os.environ.get('GPIOZERO_PIN_FACTORY') == 'mock':
            self.get_logger().warn('Running in MOCK mode.')
            Device.pin_factory = MockFactory()

        try:
            self.sensor = DistanceSensor(
                echo=echo,
                trigger=trig,
                max_distance=self.max_dist
            )
            self.get_logger().info(f'HC-SR04 initialized (Trig={trig}, Echo={echo})')
        except Exception as e:
            self.get_logger().error(f'Failed to init sensor: {e}')
            self.sensor = None

        # 4. Publisher & Timer
        self.pub = self.create_publisher(Range, '~/range', 10)
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)

    def timer_callback(self):
        if not self.sensor:
            return

        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        
        # HC-SR04 specs
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = self.fov
        msg.min_range = 0.02 # 2cm
        msg.max_range = self.max_dist

        # Read distance (gpiozero returns value in meters)
        # It handles timeouts internally and returns 1.0 (scaled) if out of range, 
        # but we should check documentation. Actually gpiozero returns value between 0 and max_distance.
        # If it fails/timeouts, it might return None or max_distance depending on version.
        
        try:
            dist = self.sensor.distance
            msg.range = float(dist)
            self.pub.publish(msg)
        except Exception as e:
            self.get_logger().debug(f'Read error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SonarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.sensor:
            node.sensor.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
