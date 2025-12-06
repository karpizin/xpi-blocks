import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from gpiozero import DigitalOutputDevice, Device
from gpiozero.pins.mock import MockFactory
import os

class RelayNode(Node):
    """
    A simple Tier 1 Block for controlling a Relay or LED.
    Subscribes to '~/cmd' (std_msgs/Bool) to toggle the state.
    """

    def __init__(self):
        super().__init__('relay_node')

        # 1. Declare Parameters
        self.declare_parameter('gpio_pin', 17)
        self.declare_parameter('active_high', True)
        self.declare_parameter('initial_value', False)
        self.declare_parameter('mock_hardware', False)

        # 2. Read Parameters
        self.pin = self.get_parameter('gpio_pin').value
        active_high = self.get_parameter('active_high').value
        initial_value = self.get_parameter('initial_value').value
        mock_mode = self.get_parameter('mock_hardware').value

        # 3. Initialize Hardware (with Mock support)
        if mock_mode or os.environ.get('GPIOZERO_PIN_FACTORY') == 'mock':
            self.get_logger().warn('Running in MOCK mode. No real GPIO will be toggled.')
            Device.pin_factory = MockFactory()
        
        try:
            self.device = DigitalOutputDevice(
                pin=self.pin,
                active_high=active_high,
                initial_value=initial_value
            )
            self.get_logger().info(f'Relay/LED initialized on GPIO {self.pin}')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize GPIO: {e}')
            # In a real deployment, we might want to exit, but for now we stay alive to log errors
            self.device = None

        # 4. Create Subscriber
        self.subscription = self.create_subscription(
            Bool,
            '~/cmd',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        if self.device:
            if msg.data:
                self.device.on()
                self.get_logger().info(f'GPIO {self.pin} -> ON')
            else:
                self.device.off()
                self.get_logger().info(f'GPIO {self.pin} -> OFF')
        else:
            self.get_logger().warn('Hardware not initialized, ignoring command.')

def main(args=None):
    rclpy.init(args=args)
    node = RelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.device:
            node.device.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
