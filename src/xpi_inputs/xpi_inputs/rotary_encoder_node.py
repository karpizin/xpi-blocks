#!/usr/bin/env python3
import time
from gpiozero import DigitalInputDevice
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32

class RotaryEncoderNode(Node):
    def __init__(self):
        super().__init__('rotary_encoder_node')
        
        # Parameters
        self.declare_parameter('pin_a', 17)
        self.declare_parameter('pin_b', 27)
        self.declare_parameter('reverse', False)
        self.declare_parameter('publish_rate', 20.0)

        self.pin_a = self.get_parameter('pin_a').value
        self.pin_b = self.get_parameter('pin_b').value
        self.reverse = self.get_parameter('reverse').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # State
        self.ticks = 0
        self.last_ticks = 0
        self.last_time = self.get_clock().now()

        # Hardware Init (gpiozero)
        try:
            self.encoder_a = DigitalInputDevice(self.pin_a, pull_up=True)
            self.encoder_b = DigitalInputDevice(self.pin_b, pull_up=True)
            
            # Attach interrupts
            self.encoder_a.when_activated = self._pulse
            self.encoder_a.when_deactivated = self._pulse
            self.encoder_b.when_activated = self._pulse
            self.encoder_b.when_deactivated = self._pulse
            
            self.get_logger().info(f"Rotary Encoder initialized on GPIO {self.pin_a} & {self.pin_b}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to init GPIO: {e}")
            return

        # Publishers
        self.tick_pub = self.create_publisher(Int32, '~/ticks', 10)
        self.vel_pub = self.create_publisher(Float32, '~/velocity', 10)

        # Timer for publishing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

    def _pulse(self):
        """
        Simple X2/X4 decoding logic.
        Read A and B. Determine direction.
        """
        a = self.encoder_a.value
        b = self.encoder_b.value
        
        # Determine direction based on state transition
        # This is a simplified logic. For robust high-speed quadrature, 
        # C++ or pigpio daemon is preferred. But this works for reasonable speeds on Pi 4.
        
        # We assume this is called on change.
        # If A changed: if A==B then dir=-1 else dir=1
        # BUT we need to know WHICH pin triggered. gpiozero calls this generically.
        # Actually, standard "when_activated" doesn't pass the pin.
        # Let's rely on a simpler 'step' logic or just read current state.
        
        # Reliable Python logic usually requires tracking last state.
        # Let's simplify: Only counting rising edge of A for X1 encoding is safest in Python.
        # But we attached to both.
        
        # Let's try "State Machine" approach if possible, but keeping it stateless is faster.
        pass 

    # Re-implementing __init__ part for better logic separation
    # Using a closure or separate methods for A and B is better.

class BetterRotaryEncoderNode(Node):
    def __init__(self):
        super().__init__('rotary_encoder_node')
        
        self.declare_parameter('pin_a', 17)
        self.declare_parameter('pin_b', 27)
        self.declare_parameter('reverse', False)
        self.declare_parameter('publish_rate', 20.0)

        self.pin_a = self.get_parameter('pin_a').value
        self.pin_b = self.get_parameter('pin_b').value
        self.reverse = self.get_parameter('reverse').value
        self.publish_rate = self.get_parameter('publish_rate').value

        self.ticks = 0
        self.last_ticks = 0
        self.last_time = self.get_clock().now()

        # Use gpiozero
        self.device_a = DigitalInputDevice(self.pin_a, pull_up=True)
        self.device_b = DigitalInputDevice(self.pin_b, pull_up=True)
        
        # Strategy: Count on Rising Edge of A (X1 Encoding) - Less CPU, stable enough
        # Direction determined by B state.
        self.device_a.when_activated = self._on_a_rising

        self.tick_pub = self.create_publisher(Int32, '~/ticks', 10)
        self.vel_pub = self.create_publisher(Float32, '~/velocity', 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        self.get_logger().info(f"Rotary Encoder (X1 mode) on GPIO {self.pin_a}/{self.pin_b}")

    def _on_a_rising(self):
        # A rose. Check B.
        # If B is Low, Clockwise. If B is High, CCW. (Standard quadrature)
        if self.device_b.value == 0:
            delta = 1
        else:
            delta = -1
            
        if self.reverse:
            delta *= -1
            
        self.ticks += delta

    def timer_callback(self):
        # Publish Ticks
        msg = Int32()
        msg.data = self.ticks
        self.tick_pub.publish(msg)
        
        # Calculate Velocity
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        
        if dt > 0:
            d_ticks = self.ticks - self.last_ticks
            vel = d_ticks / dt
            
            v_msg = Float32()
            v_msg.data = float(vel)
            self.vel_pub.publish(v_msg)
            
            self.last_ticks = self.ticks
            self.last_time = now

def main(args=None):
    rclpy.init(args=args)
    node = BetterRotaryEncoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
