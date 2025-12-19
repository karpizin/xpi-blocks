#!/usr/bin/env python3
import math
import smbus2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField
from std_msgs.msg import Float32

class QMC5883LNode(Node):
    # QMC5883L I2C Address
    ADDR = 0x0D
    
    # Registers
    REG_X_LSB = 0x00
    REG_STATUS = 0x06
    REG_CONFIG_1 = 0x09 # Control Register 1
    REG_CONFIG_2 = 0x0A # Control Register 2 (Reset)
    REG_RESET = 0x0B # SET/RESET Period
    
    def __init__(self):
        super().__init__('qmc5883l_node')
        
        # Parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('polling_rate', 10.0)
        self.declare_parameter('declination', 0.0) # In radians
        
        self.bus_id = self.get_parameter('i2c_bus').value
        self.rate = self.get_parameter('polling_rate').value
        self.declination = self.get_parameter('declination').value

        # Publishers
        self.mag_pub = self.create_publisher(MagneticField, '~/magnetic_field', 10)
        self.heading_pub = self.create_publisher(Float32, '~/heading', 10)

        # Hardware Init
        try:
            self.bus = smbus2.SMBus(self.bus_id)
            # 1. Reset
            self.bus.write_byte_data(self.ADDR, self.REG_CONFIG_2, 0x80)
            time.sleep(0.1)
            # 2. Set/Reset Period
            self.bus.write_byte_data(self.ADDR, self.REG_RESET, 0x01)
            # 3. Mode: Continuous (0x01), ODR: 50Hz (0x04), Range: 2G (0x00), OSR: 512 (0x00)
            # 0x01 | 0x04 | 0x00 | 0x00 = 0x0D
            self.bus.write_byte_data(self.ADDR, self.REG_CONFIG_1, 0x0D)
            
            self.get_logger().info(f"QMC5883L Magnetometer Initialized on Bus {self.bus_id}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize QMC5883L: {e}")
            return

        # Timer
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)

    def timer_callback(self):
        try:
            # Check status (Data Ready)
            status = self.bus.read_byte_data(self.ADDR, self.REG_STATUS)
            if not (status & 0x01):
                return

            # Read 6 bytes (X, Y, Z)
            data = self.bus.read_i2c_block_data(self.ADDR, self.REG_X_LSB, 6)
            
            # Convert to signed 16-bit
            x = self.to_int16(data[0], data[1])
            y = self.to_int16(data[2], data[3])
            z = self.to_int16(data[4], data[5])
            
            # 1. Publish Raw Magnetic Field
            msg = MagneticField()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "mag_link"
            # Scale for 2G range: approx 1/3000 Gauss per bit. 1 Gauss = 10^-4 Tesla.
            scale = 1.0 / 3000.0 * 1e-4
            msg.magnetic_field.x = float(x) * scale
            msg.magnetic_field.y = float(y) * scale
            msg.magnetic_field.z = float(z) * scale
            self.mag_pub.publish(msg)
            
            # 2. Calculate Heading
            heading = math.atan2(y, x) + self.declination
            
            # Correct for wrap around
            if heading < 0: heading += 2 * math.pi
            if heading > 2 * math.pi: heading -= 2 * math.pi
            
            # Convert to degrees
            heading_deg = math.degrees(heading)
            
            self.heading_pub.publish(Float32(data=heading_deg))

        except Exception as e:
            self.get_logger().warn(f"Read error: {e}")

    def to_int16(self, lsb, msb):
        val = (msb << 8) | lsb
        if val > 32767:
            val -= 65536
        return val

def main(args=None):
    import time
    rclpy.init(args=args)
    node = QMC5883LNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
