#!/usr/bin/env python3
import math
import smbus2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Illuminance

class MAX44009Node(Node):
    # Registers
    REG_INT_STATUS  = 0x00
    REG_INT_ENABLE  = 0x01
    REG_CONFIG      = 0x02
    REG_LUX_HIGH    = 0x03
    REG_LUX_LOW     = 0x04
    
    def __init__(self):
        super().__init__('max44009_node')
        
        # Parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x4A)
        self.declare_parameter('polling_rate', 5.0)

        self.i2c_bus_id = self.get_parameter('i2c_bus').value
        self.i2c_addr = self.get_parameter('i2c_address').value
        self.polling_rate = self.get_parameter('polling_rate').value

        # Publishers
        self.lux_pub = self.create_publisher(Illuminance, '~/illuminance', 10)

        # Hardware Init
        try:
            self.bus = smbus2.SMBus(self.i2c_bus_id)
            
            # Configure: Auto range (Default is usually fine, but let's be explicit if needed)
            # Default config: Continuous mode, Auto range.
            # No specific write needed for default auto-range behavior on power up.
            
            self.get_logger().info(f"MAX44009 Initialized at 0x{self.i2c_addr:02X} on Bus {self.i2c_bus_id}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MAX44009: {e}")
            return

        # Timer
        self.timer = self.create_timer(1.0 / self.polling_rate, self.timer_callback)

    def timer_callback(self):
        try:
            # Read 2 bytes starting from REG_LUX_HIGH
            data = self.bus.read_i2c_block_data(self.i2c_addr, self.REG_LUX_HIGH, 2)
            
            exponent = (data[0] >> 4) & 0x0F
            mantissa = ((data[0] & 0x0F) << 4) | (data[1] & 0x0F)
            
            # Formula from datasheet: Lux = 2^exponent * mantissa * 0.045
            lux = (2 ** exponent) * mantissa * 0.045
            
            msg = Illuminance()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "max44009_frame"
            msg.illuminance = float(lux)
            msg.variance = 0.0 # Unknown
            
            self.lux_pub.publish(msg)

        except Exception as e:
            self.get_logger().warning(f"Error reading MAX44009: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MAX44009Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
