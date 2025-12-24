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
        if self.bus.mock_mode: return
        
        # Stop periodic measurement first to ensure clean state
        # Command 0x3F86
        self.bus.write_i2c_block_data(self.address, 0x3F, [0x86])
        time.sleep(0.5)
        
        # Start periodic measurement
        # Command 0x21B1
        self.bus.write_i2c_block_data(self.address, 0x21, [0xB1])
        self.get_logger().info("SCD4x started periodic measurement mode.")

    def read_sensor(self):
        if self.bus.mock_mode:
            t = (time.monotonic() - self.mock_time) % 60.0
            co2 = 400 + 600 * math.sin(t * math.pi / 20.0) # 400-1000 ppm
            temp = 22.0 + 2.0 * math.cos(t * math.pi / 15.0)
            hum = 0.45 + 0.1 * math.sin(t * math.pi / 10.0)
            return int(co2), temp, hum

        # Read measurement
        # Command 0xEC05
        self.bus.write_i2c_block_data(self.address, 0xEC, [0x05])
        time.sleep(0.01)
        
        # Read 9 bytes: CO2(2)+CRC(1), Temp(2)+CRC(1), Hum(2)+CRC(1)
        data = self.bus.read_i2c_block_data(self.address, 0x00, 9)
        
        # CO2
        if self.calculate_crc(data[0:2]) != data[2]:
            raise Exception("SCD4x: CO2 CRC error")
        co2 = (data[0] << 8) | data[1]

        # Temp
        if self.calculate_crc(data[3:5]) != data[5]:
            raise Exception("SCD4x: Temp CRC error")
        raw_temp = (data[3] << 8) | data[4]
        temperature = -45.0 + 175.0 * (raw_temp / 65535.0)

        # Hum
        if self.calculate_crc(data[6:8]) != data[8]:
            raise Exception("SCD4x: Hum CRC error")
        raw_hum = (data[6] << 8) | data[7]
        humidity = (raw_hum / 65535.0)

        return co2, temperature, humidity

    def timer_callback(self):
        try:
            co2, temp, hum = self.read_sensor()
            
            current_time = self.get_clock().now().to_msg()
            
            # CO2
            co2_msg = Int32()
            co2_msg.data = int(co2)
            self.co2_pub.publish(co2_msg)

            # Temp
            temp_msg = Temperature()
            temp_msg.header.stamp = current_time
            temp_msg.header.frame_id = self.frame_id
            temp_msg.temperature = float(temp)
            self.temp_pub.publish(temp_msg)
            
            # Hum
            hum_msg = RelativeHumidity()
            hum_msg.header.stamp = current_time
            hum_msg.header.frame_id = self.frame_id
            hum_msg.relative_humidity = float(hum)
            self.hum_pub.publish(hum_msg)
            
            self.get_logger().debug(f'SCD4x: CO2={co2}ppm, T={temp:.2f}C, H={hum*100:.1f}%')
            
        except Exception as e:
            self.get_logger().error(f'SCD4x: Error reading sensor: {e}')

    def destroy_node(self):
        if not self.bus.mock_mode:
            # Stop periodic measurement on shutdown
            try:
                self.bus.write_i2c_block_data(self.address, 0x3F, [0x86])
            except: pass
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
