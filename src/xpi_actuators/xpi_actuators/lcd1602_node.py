#!/usr/bin/env python3
from RPLCD.i2c import CharLCD
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty

class LCD1602Node(Node):
# ... (intermediate code)
    def cb_line1(self, msg):
        if not self.lcd:
            return
        try:
# ... (intermediate code)
    def cb_line2(self, msg):
        if not self.lcd:
            return
        try:
# ... (intermediate code)
    def cb_write(self, msg):
        if not self.lcd:
            return
        try:
# ... (intermediate code)
    def cb_clear(self, msg):
        if not self.lcd:
            return
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
