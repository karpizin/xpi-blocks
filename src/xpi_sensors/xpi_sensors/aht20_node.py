import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature, RelativeHumidity
from xpi_commons.i2c_helper import get_smbus
import time
import math

class AHT20Node(Node):
    """
    ROS2 Node for AHT10/AHT20 temperature and humidity sensors.
    Publishes sensor_msgs/Temperature and sensor_msgs/RelativeHumidity.
    """

    def __init__(self):
        super().__init__('aht20_node')

        # Parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x38) # Standard address for AHT10/20
        self.declare_parameter('publish_rate', 1.0) # Hz
        self.declare_parameter('frame_id', 'aht20_link')
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
            self.get_logger().info(f'AHT20 initialized at 0x{self.address:02X} on bus {self.bus_id}.')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize AHT20: {e}. Falling back to mock.')
            mock_mode = True

        if mock_mode:
            self.get_logger().warn('AHT20: Running in MOCK mode.')
            self.mock_time = time.monotonic()

        # Publishers
        self.temp_pub = self.create_publisher(Temperature, '~/temperature', 10)
        self.hum_pub = self.create_publisher(RelativeHumidity, '~/humidity', 10)

        # Timer
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

    def init_sensor(self):
        if self.bus.mock_mode: return
        
        # Calibration command
        self.bus.write_i2c_block_data(self.address, 0xBE, [0x08, 0x00])
        time.sleep(0.01)

    def read_sensor(self):
        if self.bus.mock_mode:
            t = (time.monotonic() - self.mock_time) % 60.0
            temp = 20.0 + 5.0 * math.sin(t * math.pi / 15.0)
            hum = 0.5 + 0.2 * math.cos(t * math.pi / 20.0)
            return temp, hum

        # Trigger measurement
        self.bus.write_i2c_block_data(self.address, 0xAC, [0x33, 0x00])
        time.sleep(0.08) # Wait for measurement (min 80ms)

        # Read 6 bytes
        data = self.bus.read_i2c_block_data(self.address, 0x00, 6)
        
        # Calculate humidity (20 bits)
        # data[1], data[2], high 4 bits of data[3]
        raw_hum = ((data[1] << 12) | (data[2] << 4) | (data[3] >> 4))
        humidity = (raw_hum / 1048576.0) # Relative humidity [0, 1]

        # Calculate temperature (20 bits)
        # low 4 bits of data[3], data[4], data[5]
        raw_temp = ((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5]
        temperature = (raw_temp / 1048576.0) * 200.0 - 50.0

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
            
            self.get_logger().debug(f'AHT20: T={temp:.2f}C, H={hum*100:.1f}%')
            
        except Exception as e:
            self.get_logger().error(f'AHT20: Error reading sensor: {e}')

    def destroy_node(self):
        self.bus.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AHT20Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
