#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import evdev
from evdev import ecodes, InputDevice, list_devices
import threading
import time
import math

class MouseNode(Node):
    def __init__(self):
        super().__init__('mouse_node')

        # Parameters
        self.declare_parameter('device_name', 'Mouse') # Substring to match
        self.declare_parameter('device_path', '')      # Specific path override (/dev/input/eventX)
        self.declare_parameter('mode', 'velocity')     # 'velocity' (auto-center) or 'position' (accumulate)
        self.declare_parameter('sensitivity', 0.005)   # Sensitivity multiplier
        self.declare_parameter('decay', 0.2)           # Velocity decay per cycle (0.0 - 1.0)
        self.declare_parameter('poll_rate', 20.0)      # Hz for publishing and decay

        self.dev_name_filter = self.get_parameter('device_name').value
        self.dev_path = self.get_parameter('device_path').value
        self.mode = self.get_parameter('mode').value
        self.sensitivity = self.get_parameter('sensitivity').value
        self.decay = self.get_parameter('decay').value
        self.rate = self.get_parameter('poll_rate').value

        # State
        self.axes = [0.0] * 4 # X, Y, WheelV, WheelH
        self.buttons = [0] * 5 # Left, Right, Middle, Side, Extra
        self.lock = threading.Lock()
        
        # Accumulators for Velocity Mode (Instantaneous speed)
        self.rel_x = 0
        self.rel_y = 0
        self.rel_wheel = 0
        
        # Accumulators for Position Mode (Absolute position 0..1)
        self.pos_x = 0.0
        self.pos_y = 0.0

        # Find Device
        self.device = self.find_device()
        if not self.device:
            self.get_logger().error(f"No mouse device found matching '{self.dev_name_filter}'")
            self.destroy_node()
            return

        self.get_logger().info(f"Connected to: {self.device.name} ({self.device.path})")

        # Start Input Thread
        self.thread = threading.Thread(target=self.input_loop, daemon=True)
        self.thread.start()

        # Publisher & Timer
        self.pub_joy = self.create_publisher(Joy, 'joy', 10)
        self.timer = self.create_timer(1.0 / self.rate, self.update_and_publish)

    def find_device(self):
        if self.dev_path:
            try:
                return InputDevice(self.dev_path)
            except:
                return None
        
        # Auto-detect
        devices = [InputDevice(path) for path in list_devices()]
        for dev in devices:
            if self.dev_name_filter.lower() in dev.name.lower():
                return dev
        return None

    def input_loop(self):
        # Blocking read loop
        try:
            for event in self.device.read_loop():
                with self.lock:
                    if event.type == ecodes.EV_REL:
                        if event.code == ecodes.REL_X:
                            self.rel_x += event.value
                        elif event.code == ecodes.REL_Y:
                            self.rel_y += event.value # Mouse Y is usually Down=Positive
                        elif event.code == ecodes.REL_WHEEL:
                            self.rel_wheel += event.value
                    
                    elif event.type == ecodes.EV_KEY:
                        # BTN_LEFT=272, BTN_RIGHT=273, BTN_MIDDLE=274
                        # Map to 0, 1, 2
                        val = 1 if event.value > 0 else 0 # 1=Press, 0=Release, 2=Repeat
                        
                        if event.code == ecodes.BTN_LEFT:   self.buttons[0] = val
                        elif event.code == ecodes.BTN_RIGHT: self.buttons[1] = val
                        elif event.code == ecodes.BTN_MIDDLE: self.buttons[2] = val
                        elif event.code == ecodes.BTN_SIDE:   self.buttons[3] = val
                        elif event.code == ecodes.BTN_EXTRA:  self.buttons[4] = val

        except Exception as e:
            self.get_logger().error(f"Device read error: {e}")

    def update_and_publish(self):
        joy = Joy()
        joy.header.stamp = self.get_clock().now().to_msg()
        
        with self.lock:
            # Process Axes based on Mode
            
            if self.mode == 'velocity':
                # Convert raw accumulated deltas to "speed" then decay
                # X Axis
                self.axes[0] = self.rel_x * self.sensitivity
                self.axes[1] = -(self.rel_y * self.sensitivity) # Invert Y for Joystick standard (Up=Positive)
                
                # Apply Decay (Simulate spring return)
                self.rel_x *= (1.0 - self.decay)
                self.rel_y *= (1.0 - self.decay)
                
                # Zero out if small
                if abs(self.rel_x) < 0.1: self.rel_x = 0
                if abs(self.rel_y) < 0.1: self.rel_y = 0

            elif self.mode == 'position':
                # Accumulate indefinitely, clamped to -1.0 to 1.0
                self.pos_x += self.rel_x * self.sensitivity
                self.pos_y -= self.rel_y * self.sensitivity # Invert Y
                
                # Clamp
                self.pos_x = max(-1.0, min(1.0, self.pos_x))
                self.pos_y = max(-1.0, min(1.0, self.pos_y))
                
                self.axes[0] = self.pos_x
                self.axes[1] = self.pos_y
                
                # Reset deltas after consuming
                self.rel_x = 0
                self.rel_y = 0

            # Wheel always acts momentary/velocity like
            self.axes[2] = self.rel_wheel
            self.rel_wheel = 0 # Reset wheel immediately

            joy.axes = list(self.axes)
            joy.buttons = list(self.buttons)

        self.pub_joy.publish(joy)

def main(args=None):
    rclpy.init(args=args)
    node = MouseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
