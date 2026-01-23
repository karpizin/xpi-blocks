import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import Temperature, RelativeHumidity
from xpi_commons.i2c_helper import get_smbus
import time
import math

class SCD4xNode(Node):
    """
    ROS2 Node for SCD40/SCD41 CO2, Temperature and Humidity sensors.
    Publishes CO2 (ppm), sensor_msgs/Temperature and sensor_msgs/RelativeHumidity.
    """

    def __init__(self):
        super().__init__('scd4x_node')

        # Parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x62) # Standard address
        self.declare_parameter('publish_rate', 0.2) # Hz (Sensor updates every 5s in periodic mode)
        self.declare_parameter('frame_id', 'scd4x_link')
        self.declare_parameter('mock_hardware', False)

        self.bus_id = self.get_parameter('i2c_bus').value
        self.address = self.get_parameter('i2c_address').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        mock_mode = self.get_parameter('mock_hardware').value

        # Init I2C
        self.bus = get_smbus(self.bus_id, mock=mock_mode)

        try:
            self.init_sensor()
            self.get_logger().info(f'SCD4x initialized at 0x{self.address:02X} on bus {self.bus_id}.')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize SCD4x: {e}. Falling back to mock.')
            mock_mode = True

        if mock_mode:
            self.get_logger().warn('SCD4x: Running in MOCK mode.')
            self.mock_time = time.monotonic()

        # Publishers
        self.co2_pub = self.create_publisher(Int32, '~/co2', 10)
        self.temp_pub = self.create_publisher(Temperature, '~/temperature', 10)
        self.hum_pub = self.create_publisher(RelativeHumidity, '~/humidity', 10)

        # Timer
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

    def calculate_crc(self, data):
        """8-bit CRC algorithm for Sensirion sensors."""
        crc = 0xFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x31
                else:
                    crc <<= 1
                crc &= 0xFF
        return crc

    def init_sensor(self):
        if self.bus.mock_mode:
            return
        
        # Stop periodic measurement first to ensure clean state
# ... (intermediate code)
    def destroy_node(self):
        if not self.bus.mock_mode:
            # Stop periodic measurement on shutdown
            try:
                self.bus.write_i2c_block_data(self.address, 0x3F, [0x86])
            except Exception:
                pass
        self.bus.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SCD4xNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
