#!/usr/bin/env python3
import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

# Key Mappings
# WASD for Axes
AXIS_KEYS = {
    'w': (1, 1.0),   # Axis 1 (Left Y), +1.0
    's': (1, -1.0),  # Axis 1, -1.0
    'a': (3, 1.0),   # Axis 3 (Right X), +1.0 (Left turn)
    'd': (3, -1.0),  # Axis 3, -1.0 (Right turn)
    'i': (1, 1.0),   # Alternative arrows
    'k': (1, -1.0),
    'j': (3, 1.0),
    'l': (3, -1.0),
}

# 1-9, Space, Enter for Buttons
BUTTON_KEYS = {
    ' ': 0, # Space -> Button 0 (X)
    '\r': 1, # Enter -> Button 1 (O)
    'e': 2, # Triangle
    'q': 3, # Square
    '1': 4, # L1
    '2': 5, # R1
    '3': 6,
    '4': 7
}

class KeyboardToJoyNode(Node):
    def __init__(self):
        super().__init__('keyboard_to_joy_node')
        self.pub = self.create_publisher(Joy, 'joy', 10)
        self.timer = self.create_timer(0.1, self.publish_joy)
        
        self.settings = termios.tcgetattr(sys.stdin)
        self.axes = [0.0] * 8
        self.buttons = [0] * 12
        self.status = 0

        print(self.msg())

    def msg(self):
        return """
Reading from the keyboard and Publishing to /joy!
---------------------------
Moving around:
   W    
 A   D  
   S    

Buttons:
 Space : Button 0
 Enter : Button 1
 E/Q   : Button 2/3
 1-4   : Button 4-7

CTRL-C to quit
"""

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def publish_joy(self):
        key = self.getKey()
        
        # Reset axes to 0 (simulate centering spring) if no key pressed?
        # Or keep state? Usually keyboard drive implies "hold to move".
        # If we rely on getKey with timeout, we can detect release.
        
        # Simple Logic: 
        # If key pressed -> Set Axis/Button
        # If no key -> Reset all?
        # This is tricky with single key read.
        # Better approach: We decay values if no key match.
        
        if key in AXIS_KEYS:
            idx, val = AXIS_KEYS[key]
            self.axes[idx] = val
        elif key in BUTTON_KEYS:
            idx = BUTTON_KEYS[key]
            self.buttons[idx] = 1
        elif key == '\x03': # Ctrl-C
            raise KeyboardInterrupt
        else:
            # Decay / Reset logic
            # This is a naive implementation: it resets immediately if key is not matched.
            # Real keyboard drivers usually track key_down / key_up events, 
            # but that requires X11/Input access. 
            # With stdin, we only get key chars.
            # So "Hold" is repeated characters.
            self.axes = [0.0] * 8
            self.buttons = [0] * 12

        joy = Joy()
        joy.header.stamp = self.get_clock().now().to_msg()
        joy.axes = self.axes
        joy.buttons = self.buttons
        self.pub.publish(joy)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardToJoyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        # Restore terminal
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)

if __name__ == '__main__':
    main()
