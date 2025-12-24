import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import TwistStamped, Point
from sensor_msgs.msg import Range
import time
import math

# Try to import the hardware library
try:
    from pmw3901 import PMW3901, PAA5100
    HAS_HARDWARE = True
except ImportError:
    HAS_HARDWARE = False

class OpticalFlowNode(Node):
    """
    ROS2 Node for the PMW3901 Optical Flow sensor.
    Publishes velocity (TwistStamped) and integrated displacement (Point).
    Can subscribe to a Range topic for real-time height compensation.
    """

    def __init__(self):
        super().__init__('optical_flow_node')

        # Parameters
        self.declare_parameter('spi_port', 0)
        self.declare_parameter('spi_cs', 0)
        self.declare_parameter('publish_rate', 50.0) # Hz
        self.declare_parameter('default_height', 0.1) # 10cm default height
        self.declare_parameter('flow_scalar', 242.0) # Constant for PMW3901
        self.declare_parameter('mock_hardware', not HAS_HARDWARE)

        self.spi_port = self.get_parameter('spi_port').value
        self.spi_cs = self.get_parameter('spi_cs').value
        self.rate = self.get_parameter('publish_rate').value
        self.current_height = self.get_parameter('default_height').value
        self.scalar = self.get_parameter('flow_scalar').value
        self.mock_mode = self.get_parameter('mock_hardware').value

        # State
        self.total_x = 0.0
        self.total_y = 0.0
        self.last_time = time.monotonic()

        # Hardware Setup
        if self.mock_mode:
            self.get_logger().warn('PMW3901: Running in MOCK mode.')
        else:
            if not HAS_HARDWARE:
                self.get_logger().error('pmw3901 library not found. Install with: pip install pmw3901')
                raise RuntimeError('Library missing')
            
            try:
                self.sensor = PMW3901(spi_port=self.spi_port, spi_cs=self.spi_cs)
                self.sensor.set_rotation(0)
                self.get_logger().info(f'PMW3901 initialized on SPI{self.spi_port} CS{self.spi_cs}.')
            except Exception as e:
                self.get_logger().error(f'Failed to initialize PMW3901 hardware: {e}')
                self.mock_mode = True

        # Publishers
        self.vel_pub = self.create_publisher(TwistStamped, '~/velocity', 10)
        self.pos_pub = self.create_publisher(Point, '~/displacement', 10)
        self.raw_pub = self.create_publisher(Int32MultiArray, '~/raw_flow', 10)

        # Subscribers
        self.range_sub = self.create_subscription(Range, '/vl53l1x/range', self.range_callback, 10)

        # Timer
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)

    def range_callback(self, msg):
        """Update the height context if a range sensor is available."""
        if not math.isnan(msg.range):
            self.current_height = msg.range

    def timer_callback(self):
        try:
            now = time.monotonic()
            dt = now - self.last_time
            self.last_time = now

            if self.mock_mode:
                # Simulate some velocity based on sine waves
                dx, dy = int(5 * math.sin(now)), int(2 * math.cos(now))
            else:
                dx, dy = self.sensor.get_motion() # Returns raw pixel delta since last read

            # 1. Publish Raw Flow
            raw_msg = Int32MultiArray()
            raw_msg.data = [int(dx), int(dy)]
            self.raw_pub.publish(raw_msg)

            # 2. Convert to Metric Distance
            # Distance = (Pixels * Height) / Scalar
            dist_x = (dx * self.current_height) / self.scalar
            dist_y = (dy * self.current_height) / self.scalar

            # 3. Integrate Displacement
            self.total_x += dist_x
            self.total_y += dist_y

            pos_msg = Point()
            pos_msg.x = self.total_x
            pos_msg.y = self.total_y
            pos_msg.z = self.current_height
            self.pos_pub.publish(pos_msg)

            # 4. Calculate Velocity (m/s)
            vel_msg = TwistStamped()
            vel_msg.header.stamp = self.get_clock().now().to_msg()
            vel_msg.header.frame_id = 'base_link'
            vel_msg.twist.linear.x = dist_x / dt
            vel_msg.twist.linear.y = dist_y / dt
            self.vel_pub.publish(vel_msg)

            self.get_logger().debug(f'Flow dx={dx}, dy={dy} | Pos: {self.total_x:.3f}, {self.total_y:.3f}m')

        except Exception as e:
            self.get_logger().error(f'Error reading PMW3901: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = OpticalFlowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
