#!/usr/bin/env python3
import time
import board
import busio
from adafruit_ina219 import INA219, ADCResolution, BusVoltageRange

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

class INA219Node(Node):
    def __init__(self):
        super().__init__('ina219_node')
        
        # Parameters
        self.declare_parameter('polling_rate', 10.0)
        self.declare_parameter('shunt_ohms', 0.1)
        self.declare_parameter('max_amps', 3.2) # Used for calibration calculation

        self.polling_rate = self.get_parameter('polling_rate').value
        self.shunt_ohms = self.get_parameter('shunt_ohms').value
        # Note: Adafruit library auto-calibrates based on defaults, but custom calibration might be needed for high-current shunts.
        # For this basic block, we stick to library defaults which work for standard modules (0.1 ohm, 32V, 3.2A).

        # Publishers
        self.battery_pub = self.create_publisher(BatteryState, '~/battery_state', 10)

        # Hardware Init
        try:
            i2c = board.I2C() 
            self.sensor = INA219(i2c)
            
            # Optional Config (can be exposed as params later)
            # self.sensor.bus_adc_resolution = ADCResolution.ADCRES_12BIT_32S
            
            self.get_logger().info("INA219 Power Monitor Initialized")
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize INA219: {e}")
            return

        # Timer
        self.timer = self.create_timer(1.0 / self.polling_rate, self.timer_callback)

    def timer_callback(self):
        try:
            bus_voltage = self.sensor.bus_voltage # V
            current_ma = self.sensor.current       # mA
            
            msg = BatteryState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "ina219_link"
            
            msg.voltage = float(bus_voltage)
            # ROS uses Amps. Adafruit returns mA.
            # ROS convention: Negative current is discharging (powering the robot).
            # INA219 convention depends on wiring. Assuming Vin- goes to Load, positive means current flows to load.
            # So we invert it to match ROS standard (Discharging = Negative).
            msg.current = -1.0 * (float(current_ma) / 1000.0) 
            
            msg.present = True
            msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
            msg.location = "main_battery"
            
            self.battery_pub.publish(msg)

        except Exception as e:
            self.get_logger().warning(f"Error reading INA219: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = INA219Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
