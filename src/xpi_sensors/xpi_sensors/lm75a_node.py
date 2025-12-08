import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature
from xpi_commons.i2c_helper import get_smbus
import time
import struct
import math

class LM75ANode(Node):
    """
    ROS2 Node for the LM75A digital temperature sensor with thermostat function.
    Reads temperature via I2C and publishes sensor_msgs/Temperature.
    """

    # LM75A Registers
    LM75A_REG_TEMP = 0x00
    LM75A_REG_CONF = 0x01
    LM75A_REG_THYST = 0x02
    LM75A_REG_TOS = 0x03

    def __init__(self):
        super().__init__('lm75a_node')

        # 1. Declare Parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x48) # Default for LM75A, can be 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F
        self.declare_parameter('publish_rate', 1.0) # Hz
        self.declare_parameter('frame_id', 'lm75a_link')
        self.declare_parameter('mock_hardware', False)

        # 2. Read Parameters
        self.bus_id = self.get_parameter('i2c_bus').value
        self.address = self.get_parameter('i2c_address').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        mock_mode = self.get_parameter('mock_hardware').value

        # 3. Init I2C
        self.bus = get_smbus(self.bus_id, mock=mock_mode)
        
        try:
            self.init_lm75a()
            self.get_logger().info(f'LM75A initialized at 0x{self.address:02X} on bus {self.bus_id}.')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize LM75A: {e}. Falling back to mock.')
            self.bus.close() # Ensure mock bus is used if real fails
            self.bus = get_smbus(self.bus_id, mock=True) # Re-init as mock
            mock_mode = True

        if mock_mode:
            self.get_logger().warn('LM75A: Running in MOCK mode. No real I2C data will be read.')
            self.mock_temp = 20.0
            self.mock_time = time.monotonic()

        # 4. Publisher
        self.temp_publisher = self.create_publisher(Temperature, '~/temperature', 10)
        
        # 5. Timer for publishing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        self.get_logger().info(f'LM75A: Publishing temperature at {self.publish_rate} Hz.')

    def init_lm75a(self):
        # Read a byte from configuration register to verify communication
        # Set to normal operation (default)
        try:
            config_reg = self.bus.read_byte_data(self.address, self.LM75A_REG_CONF)
            self.bus.write_byte_data(self.address, self.LM75A_REG_CONF, config_reg & 0xFC) # Clear shutdown bit (bit 0) and OS_MODE (bit 1)
            self.get_logger().debug(f"LM75A config register: 0x{config_reg:02X}")
        except Exception as e:
            raise Exception(f"Failed to read/write config register, sensor may not be present: {e}")

    def read_temperature(self):
        """Reads temperature from LM75A."""
        # Read two bytes from temperature register
        try:
            raw_temp = self.bus.read_word_data(self.address, self.LM75A_REG_TEMP)
            # LM75A returns 16-bit word, MSB first, but data is 11-bit or 9-bit depending on version/resolution
            # Default resolution is 9-bit, MSB is bit 15, LSB bit 7
            # Shift right by 7 bits for 0.5°C resolution (9-bit)
            # Shift right by 5 bits for 0.125°C resolution (11-bit)
            
            # The word is usually in big-endian, so we swap bytes
            raw_temp = ((raw_temp & 0xFF) << 8) | ((raw_temp >> 8) & 0xFF)

            # Convert to signed 9-bit (0.5 deg C resolution)
            if raw_temp & 0x8000: # Negative
                temp_c = (raw_temp >> 7) - 256.0
            else: # Positive
                temp_c = (raw_temp >> 7)
            
            # For 0.125 deg C (11-bit), shift >> 5
            # temp_c = (raw_temp >> 5) / 8.0 # For 0.125C resolution

            return temp_c * 0.5 # Convert to actual temperature (0.5 deg C per LSB)
        except Exception as e:
            self.get_logger().error(f"LM75A: Error reading raw temperature: {e}")
            return None

    def timer_callback(self):
        temp_msg = Temperature()
        
        current_time = self.get_clock().now().to_msg()
        temp_msg.header.stamp = current_time
        temp_msg.header.frame_id = self.frame_id

        if self.bus.mock_mode:
            t = (time.monotonic() - self.mock_time) % 60.0 # Cycle every minute
            temp_msg.temperature = 20.0 + 3.0 * math.sin(t * math.pi / 25.0) # 17-23 degC
            self.get_logger().debug(f'Mock: T={temp_msg.temperature:.2f}C')
            
        else:
            try:
                temperature_c = self.read_temperature()
                if temperature_c is None:
                    return

                temp_msg.temperature = temperature_c
                temp_msg.variance = 0.0 # LM75A has fixed resolution
                self.get_logger().debug(f'Real: T={temp_msg.temperature:.2f}C')

            except Exception as e:
                self.get_logger().error(f'LM75A: Error reading sensor data: {e}')
                return # Skip publishing if error

        self.temp_publisher.publish(temp_msg)

    def destroy_node(self):
        self.bus.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LM75ANode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
