#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import ParameterDescriptor
from std_msgs.msg import Float32
from gpiozero import Servo
from gpiozero.pins.factory import Factory

class ESCDriverNode(Node):
    """
    ROS2 Driver for Electronic Speed Controllers (ESC) using standard PWM (Servo protocol).
    Supports both Unidirectional (Drone) and Bidirectional (Car) ESCs.
    """
    def __init__(self):
        super().__init__('esc_driver')
        
        # Parameters
        self.declare_parameter('pin', 12, ParameterDescriptor(description='GPIO pin connected to ESC signal'))
        self.declare_parameter('min_pulse', 0.0010, ParameterDescriptor(description='Min pulse width (s), usually 1ms'))
        self.declare_parameter('max_pulse', 0.0020, ParameterDescriptor(description='Max pulse width (s), usually 2ms'))
        self.declare_parameter('frame_width', 0.020, ParameterDescriptor(description='PWM Frame width (s), usually 20ms (50Hz)'))
        self.declare_parameter('stop_value', -1.0, ParameterDescriptor(description='Value to send on timeout/stop. -1.0 for unidirectional (min), 0.0 for bidirectional (center).'))
        self.declare_parameter('timeout', 0.5, ParameterDescriptor(description='Safety stop timeout (s)'))

        self.pin = self.get_parameter('pin').value
        self.min_pulse = self.get_parameter('min_pulse').value
        self.max_pulse = self.get_parameter('max_pulse').value
        self.frame_width = self.get_parameter('frame_width').value
        self.stop_value = self.get_parameter('stop_value').value
        self.timeout = self.get_parameter('timeout').value

        self.get_logger().info(f"Configuring ESC on GPIO {self.pin}...")
        
        # Initialize Hardware
        try:
            self.esc = Servo(
                self.pin, 
                min_pulse_width=self.min_pulse, 
                max_pulse_width=self.max_pulse, 
                frame_width=self.frame_width,
                initial_value=self.stop_value
            )
        except Exception as e:
            self.get_logger().error(f"Failed to initialize ESC: {e}")
            # We don't exit to keep node alive for diagnostics, but functionality is broken
            self.esc = None
            return

        self.last_cmd_time = time.time()
        
        # Subscription
        self.sub = self.create_subscription(Float32, '~/cmd', self.cmd_callback, 10)
        
        # Safety Timer
        self.create_timer(0.1, self.safety_loop)
        
        self.get_logger().info("ESC Driver initialized. Waiting for commands.")

    def cmd_callback(self, msg):
        self.last_cmd_time = time.time()
        if self.esc:
            # Clip value to [-1, 1] as expected by gpiozero.Servo
            val = max(min(msg.data, 1.0), -1.0)
            self.esc.value = val

    def safety_loop(self):
        if self.esc:
            if time.time() - self.last_cmd_time > self.timeout:
                if self.esc.value != self.stop_value:
                    self.get_logger().warn("Command timeout. Stopping ESC.")
                    self.esc.value = self.stop_value

    def destroy_node(self):
        if hasattr(self, 'esc') and self.esc:
            self.esc.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ESCDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
