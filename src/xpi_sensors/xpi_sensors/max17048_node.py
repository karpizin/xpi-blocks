#!/usr/bin/env python3
import time
import board
import busio
import adafruit_max1704x

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

class MAX17048Node(Node):
    def __init__(self):
        super().__init__('max17048_node')
        
        # Parameters
        self.declare_parameter('polling_rate', 1.0)
        self.polling_rate = self.get_parameter('polling_rate').value

        # Publishers
        self.battery_pub = self.create_publisher(BatteryState, '~/battery_state', 10)

        # Hardware Init
        try:
            i2c = board.I2C() 
            self.sensor = adafruit_max1704x.MAX17048(i2c)
            self.get_logger().info("MAX17048 Fuel Gauge Initialized")
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize MAX17048: {e}")
            return

        # Timer
        self.timer = self.create_timer(1.0 / self.polling_rate, self.timer_callback)

    def timer_callback(self):
        try:
            voltage = self.sensor.cell_voltage
            percentage = self.sensor.cell_percent # 0-100
            
            msg = BatteryState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "battery_link"
            
            msg.voltage = float(voltage)
            msg.percentage = float(percentage) / 100.0 # ROS expects 0.0 to 1.0
            
            msg.present = True
            msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
            msg.location = "main_battery"
            
            # Status estimation based on Crate or voltage could be added here
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
            
            self.battery_pub.publish(msg)

        except Exception as e:
            self.get_logger().warning(f"Error reading MAX17048: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MAX17048Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
