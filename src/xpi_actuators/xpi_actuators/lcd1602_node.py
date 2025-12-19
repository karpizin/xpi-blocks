#!/usr/bin/env python3
import time
from RPLCD.i2c import CharLCD
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty

class LCD1602Node(Node):
    def __init__(self):
        super().__init__('lcd1602_node')
        
        # Parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x27)
        self.declare_parameter('cols', 16)
        self.declare_parameter('rows', 2)

        self.i2c_bus_id = self.get_parameter('i2c_bus').value
        self.i2c_addr = self.get_parameter('i2c_address').value
        self.cols = self.get_parameter('cols').value
        self.rows = self.get_parameter('rows').value

        # Subscribers
        self.sub_line1 = self.create_subscription(String, '~/line1', self.cb_line1, 10)
        self.sub_line2 = self.create_subscription(String, '~/line2', self.cb_line2, 10)
        self.sub_write = self.create_subscription(String, '~/write', self.cb_write, 10)
        self.sub_clear = self.create_subscription(Empty, '~/clear', self.cb_clear, 10)

        # Hardware Init
        try:
            # RPLCD automatically uses smbus
            self.lcd = CharLCD(i2c_expander='PCF8574', 
                               address=self.i2c_addr,
                               port=self.i2c_bus_id,
                               cols=self.cols, 
                               rows=self.rows,
                               dotsize=8,
                               charmap='A00',
                               auto_linebreaks=True,
                               backlight_enabled=True)
            
            self.lcd.clear()
            self.lcd.write_string('XPI-Blocks\nReady...')
            self.get_logger().info(f"LCD 1602 Initialized at 0x{self.i2c_addr:02X}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize LCD: {e}")
            self.lcd = None

    def cb_line1(self, msg):
        if not self.lcd: return
        try:
            # Pad or trim to fit
            text = msg.data[:self.cols].ljust(self.cols)
            self.lcd.cursor_pos = (0, 0)
            self.lcd.write_string(text)
        except Exception as e:
            self.get_logger().warn(f"LCD Error: {e}")

    def cb_line2(self, msg):
        if not self.lcd: return
        try:
            text = msg.data[:self.cols].ljust(self.cols)
            self.lcd.cursor_pos = (1, 0)
            self.lcd.write_string(text)
        except Exception as e:
            self.get_logger().warn(f"LCD Error: {e}")

    def cb_write(self, msg):
        if not self.lcd: return
        try:
            self.lcd.clear()
            self.lcd.write_string(msg.data)
        except Exception as e:
            self.get_logger().warn(f"LCD Error: {e}")

    def cb_clear(self, msg):
        if not self.lcd: return
        try:
            self.lcd.clear()
        except Exception as e:
            self.get_logger().warn(f"LCD Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LCD1602Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.lcd:
            node.lcd.clear()
            node.lcd.backlight_enabled = False
            node.lcd.close(clear=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
