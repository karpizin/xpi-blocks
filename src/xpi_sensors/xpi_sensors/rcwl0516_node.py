#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from gpiozero import DigitalInputDevice

class RCWL0516Node(Node):
    def __init__(self):
        super().__init__('rcwl0516_node')
        
        # Parameters
        self.declare_parameter('gpio_pin', 17)
        self.declare_parameter('pull_up', False)
        
        pin = self.get_parameter('gpio_pin').value
        pull_up = self.get_parameter('pull_up').value
        
        # Hardware setup
        try:
            self.sensor = DigitalInputDevice(pin, pull_up=pull_up)
            self.get_logger().info(f'RCWL-0516: Initialized on GPIO {pin}')
        except Exception as e:
            self.get_logger().error(f'Failed to init GPIO {pin}: {e}')
            return

        # Publisher
        self.pub = self.create_publisher(Bool, '~/motion', 10)
        
        # State tracking to avoid spamming
        self.last_state = False
        
        # Timer for polling (100Hz for fast reaction)
        self.create_timer(0.01, self.check_sensor)

    def check_state(self):
        return self.sensor.is_active

    def check_sensor(self):
        current_state = self.check_state()
        
        # Publish only on change or periodically? 
        # For motion sensors, change-based + heartbeat is better.
        if current_state != self.last_state:
            msg = Bool()
            msg.data = current_state
            self.pub.publish(msg)
            self.last_state = current_state
            
            status = "DETECTED" if current_state else "CLEARED"
            self.get_logger().info(f'Motion {status}')

def main(args=None):
    rclpy.init(args=args)
    node = RCWL0516Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
