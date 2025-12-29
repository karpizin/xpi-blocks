#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
import smbus2

class SW6106Node(Node):
    """
    ROS2 Node for SW6106 Power Bank IC (found in Waveshare Battery HAT).
    I2C Address: 0x3C
    """
    def __init__(self):
        super().__init__('sw6106_node')
        
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x3C)
        self.declare_parameter('update_rate', 1.0)
        
        self.bus_num = self.get_parameter('i2c_bus').value
        self.address = self.get_parameter('i2c_address').value
        rate = self.get_parameter('update_rate').value
        
        try:
            self.bus = smbus2.SMBus(self.bus_num)
            self.get_logger().info(f"SW6106 initialized on bus {self.bus_num} at 0x{self.address:02X}")
        except Exception as e:
            self.get_logger().error(f"Failed to open I2C bus: {e}")
            self.bus = None

        self.battery_pub = self.create_publisher(BatteryState, '~/battery', 10)
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)

    def read_word(self, reg):
        # SW6106 uses standard I2C word reading
        return self.bus.read_word_data(self.address, reg)

    def timer_callback(self):
        if not self.bus: return

        try:
            # Register 0x08: Battery capacity percentage
            percentage = self.bus.read_byte_data(self.address, 0x08)
            
            # Register 0x0A-0x0B: Voltage (in mV)
            v_raw = self.read_word(0x0A)
            voltage = float(v_raw) / 1000.0
            
            # Register 0x0C-0x0D: Current (in mA)
            i_raw = self.read_word(0x0C)
            current = float(i_raw) / 1000.0

            msg = BatteryState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.voltage = voltage
            msg.current = current
            msg.percentage = float(percentage) / 100.0
            msg.present = True
            
            # Charging status from register 0x01
            status = self.bus.read_byte_data(self.address, 0x01)
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING if (status & 0x01) else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            
            self.battery_pub.publish(msg)
        except Exception as e:
            self.get_logger().debug(f"Read error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SW6106Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
