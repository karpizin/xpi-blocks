#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool, Float32
from sensor_msgs.msg import BatteryState
import smbus2

class IP5306Node(Node):
    """
    ROS2 Node for IP5306 Power Bank IC.
    Commonly used in T-Power and other RPi Power HATs.
    I2C Address: 0x75
    """
    def __init__(self):
        super().__init__('ip5306_node')
        
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x75)
        self.declare_parameter('update_rate', 1.0) # Hz
        
        self.bus_num = self.get_parameter('i2c_bus').value
        self.address = self.get_parameter('i2c_address').value
        rate = self.get_parameter('update_rate').value
        
        try:
            self.bus = smbus2.SMBus(self.bus_num)
            self.get_logger().info(f"IP5306 initialized on bus {self.bus_num} at 0x{self.address:02X}")
        except Exception as e:
            self.get_logger().error(f"Failed to open I2C bus: {e}")
            self.bus = None

        # Publishers
        self.battery_pub = self.create_publisher(BatteryState, '~/battery', 10)
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)

    def read_battery_level(self):
        if not self.bus: return 0
        try:
            # Register 0x78 contains fuel level (0-100)
            # Some versions use different registers, but 0x78 is common for IP5306 I2C version
            level = self.bus.read_byte_data(self.address, 0x78)
            return level
        except Exception:
            return -1

    def is_charging(self):
        if not self.bus: return False
        try:
            # Register 0x70 bit 3 is charging status
            stat = self.bus.read_byte_data(self.address, 0x70)
            return bool(stat & 0x08)
        except Exception:
            return False

    def timer_callback(self):
        level = self.read_battery_level()
        if level < 0: return

        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.percentage = float(level) / 100.0
        
        charging = self.is_charging()
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING if charging else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        
        # IP5306 doesn't give precise voltage via I2C easily, usually just 4 stages or approximate %
        msg.voltage = 3.7 # Nominal
        msg.present = True
        
        self.battery_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = IP5306Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
