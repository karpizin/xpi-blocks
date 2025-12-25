#!/usr/bin/env python3
import time
import board
import busio
import adafruit_mcp4725

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class MCP4725Node(Node):
    """
    ROS2 Node for MCP4725 12-bit Digital-to-Analog Converter (DAC) via I2C.
    Outputs analog voltage between 0 and VCC (usually 3.3V or 5V).
    """
    def __init__(self):
        super().__init__('mcp4725_node')
        
        # Parameters
        self.declare_parameter('i2c_address', 0x62) # Default is 0x62, can be 0x63
        self.declare_parameter('vcc', 3.3) # Reference voltage for conversion
        
        self.address = self.get_parameter('i2c_address').value
        self.vcc = self.get_parameter('vcc').value

        # Hardware Init
        try:
            i2c = board.I2C()
            self.dac = adafruit_mcp4725.MCP4725(i2c, address=self.address)
            self.get_logger().info(f"MCP4725 DAC initialized at 0x{self.address:02X} (VCC: {self.vcc}V)")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize MCP4725: {e}")
            return

        # Subscriber
        # Expects a normalized value from 0.0 to 1.0 (0V to VCC)
        self.create_subscription(Float32, '~/cmd_normalized', self.cmd_callback, 10)
        
        # Optional: direct voltage command
        self.create_subscription(Float32, '~/cmd_voltage', self.voltage_callback, 10)

    def cmd_callback(self, msg):
        """Sets DAC output based on 0.0 to 1.0 scale"""
        val = max(0.0, min(1.0, msg.data))
        # 12-bit resolution = 0 to 4095
        raw_value = int(val * 4095)
        self._set_dac(raw_value)

    def voltage_callback(self, msg):
        """Sets DAC output based on target voltage"""
        val = max(0.0, min(self.vcc, msg.data))
        raw_value = int((val / self.vcc) * 4095)
        self._set_dac(raw_value)

    def _set_dac(self, raw_value):
        try:
            self.dac.raw_value = raw_value
            # self.get_logger().debug(f"DAC set to raw: {raw_value}")
        except Exception as e:
            self.get_logger().warn(f"DAC Write Error: {e}")

    def destroy_node(self):
        # Reset to 0V on exit for safety
        if hasattr(self, 'dac'):
            self.dac.raw_value = 0
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MCP4725Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
