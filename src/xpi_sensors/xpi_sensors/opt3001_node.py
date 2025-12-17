#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Illuminance
from xpi_commons.i2c_helper import get_smbus

class OPT3001Node(Node):
    def __init__(self):
        super().__init__('opt3001_node')

        # Parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x44)  # Default address (ADDR -> GND)
        self.declare_parameter('poll_rate', 1.0)     # Hz
        self.declare_parameter('frame_id', 'opt3001_link')

        self.i2c_bus_id = self.get_parameter('i2c_bus').value
        self.i2c_address = self.get_parameter('i2c_address').value
        self.poll_rate = self.get_parameter('poll_rate').value
        self.frame_id = self.get_parameter('frame_id').value

        # I2C Setup
        try:
            self.bus = get_smbus(self.i2c_bus_id)
            self.configure_sensor()
            self.get_logger().info(f"OPT3001 initialized on bus {self.i2c_bus_id} at address {hex(self.i2c_address)}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize OPT3001: {e}")
            self.destroy_node()
            return

        # Publishers
        self.pub_lux = self.create_publisher(Illuminance, '~/illuminance', 10)

        # Timer
        self.timer = self.create_timer(1.0 / self.poll_rate, self.read_sensor)

        # Constants
        self.REG_RESULT = 0x00
        self.REG_CONFIG = 0x01
        
        # Configuration: 
        # Range Number (15:12) = 1100 (Auto-scale)
        # Convertion Time (11) = 0 (100ms) or 1 (800ms) - let's stick to default 800ms for stability?
        # Mode of Conversion (10:9) = 11 (Continuous conversions)
        # Latch (4) = 1
        # Default config often used: 0xCC10 (Auto-range, 800ms conversion, Continuous)
        # Or 0xC410 (Auto-range, 100ms conversion, Continuous) if we want faster updates.
        # Let's use 0xCC10 (800ms integration) for better low-light performance by default.
        self.CONFIG_VALUE = 0xCC10 

    def configure_sensor(self):
        # Write configuration to register 0x01
        # OPT3001 is Big Endian for data (MSB first)
        # smbus usually writes LSB first, so we might need to swap bytes manually or use write_word_data carefully.
        # However, read/write_word_data in smbus usually handles bus endianness, but let's be explicit.
        
        # Swapping bytes for the config value
        # config_swapped = ((self.CONFIG_VALUE & 0xFF) << 8) | ((self.CONFIG_VALUE >> 8) & 0xFF)
        # self.bus.write_word_data(self.i2c_address, self.REG_CONFIG, config_swapped)
        
        # Alternative: write block data to be sure of order [MSB, LSB]
        msb = (self.CONFIG_VALUE >> 8) & 0xFF
        lsb = self.CONFIG_VALUE & 0xFF
        self.bus.write_i2c_block_data(self.i2c_address, self.REG_CONFIG, [msb, lsb])

    def read_sensor(self):
        try:
            # Read 2 bytes from Result Register (0x00)
            data = self.bus.read_i2c_block_data(self.i2c_address, self.REG_RESULT, 2)
            
            # Convert to 16-bit value (Big Endian: first byte is MSB)
            raw_val = (data[0] << 8) | data[1]
            
            # Extract Exponent (4 bits) and Mantissa (12 bits)
            exponent = (raw_val >> 12) & 0x0F
            mantissa = raw_val & 0x0FFF
            
            # Calculate Lux
            # LSB_Size = 0.01 * 2^E
            # Lux = LSB_Size * R
            lux = 0.01 * (2 ** exponent) * mantissa
            
            msg = Illuminance()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.illuminance = float(lux)
            msg.variance = 0.0 # Unknown
            
            self.pub_lux.publish(msg)
            
        except Exception as e:
            self.get_logger().warning(f"Error reading OPT3001: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = OPT3001Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
