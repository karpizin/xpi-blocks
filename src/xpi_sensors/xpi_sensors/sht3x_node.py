import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature, RelativeHumidity
from xpi_commons.i2c_helper import get_smbus
import time
import math

class SHT3xNode(Node):
    """
    ROS2 Node for SHT30/SHT31/SHT35 temperature and humidity sensors.
    Publishes sensor_msgs/Temperature and sensor_msgs/RelativeHumidity.
    """

    def __init__(self):
        super().__init__('sht3x_node')

        # Parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x44) # Default address
        self.declare_parameter('publish_rate', 1.0) # Hz
        self.declare_parameter('frame_id', 'sht3x_link')
        self.declare_parameter('mock_hardware', False)

        self.bus_id = self.get_parameter('i2c_bus').value
        self.address = self.get_parameter('i2c_address').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        mock_mode = self.get_parameter('mock_hardware').value

        # Init I2C
        self.bus = get_smbus(self.bus_id, mock=mock_mode)

        try:
            self.get_logger().info(f'SHT3x attempt to initialize at 0x{self.address:02X} on bus {self.bus_id}.')
            # Reset/Wake up check could be added here
            self.get_logger().info(f'SHT3x initialized.')
        except Exception as e:
            self.get_logger().error(f'Failed to communicate with SHT3x: {e}. Falling back to mock.')
            mock_mode = True

        if mock_mode:
            self.get_logger().warn('SHT3x: Running in MOCK mode.')
            self.mock_time = time.monotonic()

        # Publishers
        self.temp_pub = self.create_publisher(Temperature, '~/temperature', 10)
        self.hum_pub = self.create_publisher(RelativeHumidity, '~/humidity', 10)

        # Timer
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

    def read_sensor(self):
        if self.bus.mock_mode:
            t = (time.monotonic() - self.mock_time) % 60.0
            temp = 25.0 + 3.0 * math.sin(t * math.pi / 10.0)
            hum = 0.4 + 0.1 * math.cos(t * math.pi / 15.0)
            return temp, hum

        # Send measurement command: High repeatability, Clock stretching disabled
        # Command 0x2C06
        self.bus.write_i2c_block_data(self.address, 0x2C, [0x06])
        time.sleep(0.02) # Wait for measurement (max 15ms)

        # Read 6 bytes
        # [Temp MSB, Temp LSB, CRC, Hum MSB, Hum LSB, CRC]
        data = self.bus.read_i2c_block_data(self.address, 0x00, 6)
        
        # Convert temperature
        raw_temp = (data[0] << 8) | data[1]
        temperature = -45.0 + 175.0 * (raw_temp / 65535.0)

        # Convert humidity
        raw_hum = (data[3] << 8) | data[4]
        humidity = (raw_hum / 65535.0) # Relative humidity [0, 1]

        return temperature, humidity

    def timer_callback(self):
        try:
            temp, hum = self.read_sensor()
            
            current_time = self.get_clock().now().to_msg()
            
            temp_msg = Temperature()
            temp_msg.header.stamp = current_time
            temp_msg.header.frame_id = self.frame_id
            temp_msg.temperature = float(temp)
            
            hum_msg = RelativeHumidity()
            hum_msg.header.stamp = current_time
            hum_msg.header.frame_id = self.frame_id
            hum_msg.relative_humidity = float(hum)

            self.temp_pub.publish(temp_msg)
            self.hum_pub.publish(hum_msg)
            
            self.get_logger().debug(f'SHT3x: T={temp:.2f}C, H={hum*100:.1f}%')
            
        except Exception as e:
            self.get_logger().error(f'SHT3x: Error reading sensor: {e}')

    def destroy_node(self):
        self.bus.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SHT3xNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
