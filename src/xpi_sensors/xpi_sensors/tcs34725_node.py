#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Illuminance
from std_msgs.msg import ColorRGBA, Float32
from xpi_commons.i2c_helper import get_smbus
import time
import math

class TCS34725Node(Node):
    def __init__(self):
        super().__init__('tcs34725_node')

        # Parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x29)  # Default
        self.declare_parameter('poll_rate', 2.0)     # Hz
        self.declare_parameter('frame_id', 'tcs34725_link')
        self.declare_parameter('gain', 1)            # 1, 4, 16, 60
        self.declare_parameter('integration_time_ms', 154) # 2.4, 24, 50, 101, 154, 700

        self.i2c_bus_id = self.get_parameter('i2c_bus').value
        self.i2c_address = self.get_parameter('i2c_address').value
        self.poll_rate = self.get_parameter('poll_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # Mapping Params to Registers
        self.gain = self.get_parameter('gain').value
        self.integ_ms = self.get_parameter('integration_time_ms').value

        # TCS34725 Registers
        self.COMMAND_BIT = 0x80
        self.ENABLE_REG = 0x00
        self.ATIME_REG = 0x01
        self.CONTROL_REG = 0x0F
        self.ID_REG = 0x12
        self.CDATAL_REG = 0x14 # Data starts here: C, R, G, B (2 bytes each)

        self.ENABLE_PON = 0x01 # Power ON
        self.ENABLE_AEN = 0x02 # ADC Enable

        # Integration Time Map
        # 0xFF = 2.4ms, 0xF6 = 24ms, 0xEB = 50ms, 0xD5 = 101ms, 0xC0 = 154ms, 0x00 = 700ms
        self.IT_MAP = {
            2.4: 0xFF,
            24: 0xF6,
            50: 0xEB,
            101: 0xD5,
            154: 0xC0,
            700: 0x00
        }
        
        # Gain Map: 0x00=1x, 0x01=4x, 0x02=16x, 0x03=60x
        self.GAIN_MAP = {
            1: 0x00,
            4: 0x01,
            16: 0x02,
            60: 0x03
        }

        # I2C Setup
        try:
            self.bus = get_smbus(self.i2c_bus_id)
            self.check_id()
            self.configure_sensor()
            self.get_logger().info(f"TCS34725 initialized on bus {self.i2c_bus_id} at address {hex(self.i2c_address)}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize TCS34725: {e}")
            self.destroy_node()
            return

        # Publishers
        self.pub_lux = self.create_publisher(Illuminance, '~/illuminance', 10)
        self.pub_color = self.create_publisher(ColorRGBA, '~/color', 10)
        self.pub_temp = self.create_publisher(Float32, '~/color_temp', 10)

        self.timer = self.create_timer(1.0 / self.poll_rate, self.read_sensor)

    def write_reg(self, reg, value):
        self.bus.write_byte_data(self.i2c_address, self.COMMAND_BIT | reg, value)

    def read_reg(self, reg):
        return self.bus.read_byte_data(self.i2c_address, self.COMMAND_BIT | reg)

    def check_id(self):
        chip_id = self.read_reg(self.ID_REG)
        # TCS34725 should be 0x44, TCS34721 is 0x44. TCS34723/7 is 0x4D.
        if chip_id not in [0x44, 0x4D, 0x10]: # 0x10 often seen on generic clones?
             self.get_logger().warning(f"Unexpected Chip ID: {hex(chip_id)}. Proceeding anyway.")

    def configure_sensor(self):
        # Set Integration Time
        atime = self.IT_MAP.get(self.integ_ms, 0xC0) # Default 154ms
        self.write_reg(self.ATIME_REG, atime)

        # Set Gain
        gain = self.GAIN_MAP.get(self.gain, 0x00) # Default 1x
        self.write_reg(self.CONTROL_REG, gain)

        # Enable
        self.write_reg(self.ENABLE_REG, self.ENABLE_PON)
        time.sleep(0.003)
        self.write_reg(self.ENABLE_REG, self.ENABLE_PON | self.ENABLE_AEN)
        time.sleep(self.integ_ms / 1000.0 + 0.1)

    def calculate_color_temperature(self, r, g, b):
        # McCamy's formula
        if (r + g + b) == 0:
            return 0.0
        
        # RGB to Tristimulus (Approximation)
        X = -0.14282 * r + 1.54924 * g + -0.95641 * b
        Y = -0.32466 * r + 1.57837 * g + -0.73191 * b
        Z = -0.68202 * r + 0.77073 * g +  0.56332 * b

        if (X + Y + Z) == 0:
            return 0.0

        xc = X / (X + Y + Z)
        yc = Y / (X + Y + Z)
        
        if (0.1858 - yc) == 0:
            return 0.0

        n = (xc - 0.3320) / (0.1858 - yc)
        cct = 449.0 * (n ** 3) + 3525.0 * (n ** 2) + 6823.3 * n + 5520.33
        return cct

    def calculate_lux(self, r, g, b):
        # Manufacturer simplified formula
        return (-0.32466 * r) + (1.57837 * g) + (-0.73191 * b)

    def read_sensor(self):
        try:
            # Block read 8 bytes: CDATAL, CDATAH, R, R, G, G, B, B
            # 0x14 | 0x80 = 0x94 (Command register + Address)
            # Many I2C devices support auto-increment if we set the protocol bit.
            # For TCS34725, Command Bit 0x80 is standard. 
            # To read block, we typically request from 0x14.
            
            # Using smbus2 block read from command reg 0x14
            data = self.bus.read_i2c_block_data(self.i2c_address, self.COMMAND_BIT | self.CDATAL_REG, 8)
            
            c = data[1] << 8 | data[0]
            r = data[3] << 8 | data[2]
            g = data[5] << 8 | data[4]
            b = data[7] << 8 | data[6]

            if c == 0:
                return # Avoid division by zero

            # Publish Illuminance (Clear channel is raw light, but Lux requires calculation)
            # Simple Lux calculation
            lux = self.calculate_lux(r, g, b)
            msg_lux = Illuminance()
            msg_lux.header.stamp = self.get_clock().now().to_msg()
            msg_lux.header.frame_id = self.frame_id
            msg_lux.illuminance = max(0.0, float(lux))
            self.pub_lux.publish(msg_lux)

            # Publish Color Temp
            cct = self.calculate_color_temperature(r, g, b)
            msg_temp = Float32()
            msg_temp.data = float(cct)
            self.pub_temp.publish(msg_temp)

            # Publish ColorRGBA (Normalized)
            msg_color = ColorRGBA()
            msg_color.r = float(r) / c
            msg_color.g = float(g) / c
            msg_color.b = float(b) / c
            msg_color.a = 1.0 # Clear channel normalized? Or just Alpha=1
            # Actually, standard is 0..1. r/c is relative chromaticity.
            # Sometimes people want raw/255. Let's provide chromaticity (color ratio).
            self.pub_color.publish(msg_color)
            
        except Exception as e:
            self.get_logger().warning(f"Error reading TCS34725: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TCS34725Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
