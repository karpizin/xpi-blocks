import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from gpiozero import MotionSensor, Device
from gpiozero.pins.mock import MockFactory
import time

class PIRNode(Node):
    """
    ROS2 Node for a Passive Infrared (PIR) motion sensor.
    Publishes motion detection state via GPIO.
    """

    def __init__(self):
        super().__init__('pir_node')

        # Parameters
        self.declare_parameter('gpio_pin', 26)
        self.declare_parameter('publish_rate', 10.0) # Hz
        self.declare_parameter('mock_hardware', False)

        pin_num = self.get_parameter('gpio_pin').value
        publish_rate = self.get_parameter('publish_rate').value
        mock_mode = self.get_parameter('mock_hardware').value

        if mock_mode:
            Device.pin_factory = MockFactory()
            self.get_logger().warn('PIR: Running in MOCK mode.')

        # Hardware Setup
        try:
            # We use MotionSensor from gpiozero which handles edge detection
            self.pir = MotionSensor(pin_num)
            self.get_logger().info(f"PIR Sensor initialized on GPIO {pin_num}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize PIR Sensor: {e}")
            raise e

        # Publisher
        self.motion_pub = self.create_publisher(Bool, '~/motion', 10)

        # Timer
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)

    def timer_callback(self):
        try:
            msg = Bool()
            msg.data = self.pir.motion_detected
            self.motion_pub.publish(msg)
            
            if msg.data:
                self.get_logger().debug("Motion detected!")

        except Exception as e:
            self.get_logger().error(f"Error reading PIR sensor: {e}")

    def destroy_node(self):
        self.pir.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PIRNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
