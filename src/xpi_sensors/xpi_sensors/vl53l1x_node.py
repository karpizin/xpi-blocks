import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import time
import math

# Try to import the hardware library
try:
    import VL53L1X
    HAS_HARDWARE = True
except ImportError:
    HAS_HARDWARE = False

class VL53L1XNode(Node):
    """
    ROS2 Node for the VL53L1X ToF distance sensor.
    Publishes sensor_msgs/Range.
    """

    def __init__(self):
        super().__init__('vl53l1x_node')

        # Parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('publish_rate', 10.0) # Hz
        self.declare_parameter('mode', 2) # 1=Short (1.3m), 2=Long (4m)
        self.declare_parameter('frame_id', 'vl53l1x_link')
        self.declare_parameter('mock_hardware', not HAS_HARDWARE)

        self.bus_id = self.get_parameter('i2c_bus').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.mode = self.get_parameter('mode').value
        self.frame_id = self.get_parameter('frame_id').value
        self.mock_mode = self.get_parameter('mock_hardware').value

        if self.mock_mode:
            self.get_logger().warn('VL53L1X: Running in MOCK mode.')
            self.mock_time = time.monotonic()
        else:
            if not HAS_HARDWARE:
                self.get_logger().error('VL53L1X library not found. Install with: pip install vl53l1x')
                raise RuntimeError('Library missing')
            
            try:
                self.sensor = VL53L1X.VL53L1X(i2c_bus=self.bus_id, i2c_address=0x29)
                self.sensor.open()
                self.sensor.start_ranging(self.mode)
                self.get_logger().info(f'VL53L1X initialized on bus {self.bus_id} in mode {self.mode}.')
            except Exception as e:
                self.get_logger().error(f'Failed to initialize VL53L1X hardware: {e}')
                self.mock_mode = True
                self.get_logger().warn('Falling back to MOCK mode.')

        # Publisher
        self.range_pub = self.create_publisher(Range, '~/range', 10)

        # Timer
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

    def timer_callback(self):
        try:
            distance_mm = 0.0
            if self.mock_mode:
                t = time.monotonic() - self.mock_time
                distance_mm = 1000 + 500 * math.sin(t * 2 * math.pi / 5.0) # 0.5m to 1.5m oscillation
            else:
                distance_mm = self.sensor.get_distance() # mm

            # Fill Range message
            msg = Range()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.radiation_type = Range.INFRARED
            msg.field_of_view = 0.471 # 27 degrees in radians
            msg.min_range = 0.04 # 4 cm
            msg.max_range = 4.0 if self.mode == 2 else 1.3
            msg.range = distance_mm / 1000.0 # Convert mm to meters

            self.range_pub.publish(msg)
            self.get_logger().debug(f'Range: {msg.range:.3f} m')

        except Exception as e:
            self.get_logger().error(f'Error reading VL53L1X: {e}')

    def destroy_node(self):
        if not self.mock_mode and hasattr(self, 'sensor'):
            self.sensor.stop_ranging()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VL53L1XNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
