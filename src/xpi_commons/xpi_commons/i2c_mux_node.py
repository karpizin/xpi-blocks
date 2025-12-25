#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from example_interfaces.srv import SetInt32 # Using standard service for channel selection
import smbus2
import time

class I2CMuxNode(Node):
    """
    ROS2 Node for TCA9548A I2C Multiplexer.
    Enables switching between 8 downstream I2C channels.
    """

    def __init__(self):
        super().__init__('i2c_mux_node')

        # Parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x70) # Default for TCA9548A
        self.declare_parameter('default_channel', -1) # -1 means all channels disabled
        self.declare_parameter('mock_hardware', False)

        self.bus_id = self.get_parameter('i2c_bus').value
        self.address = self.get_parameter('i2c_address').value
        self.mock_mode = self.get_parameter('mock_hardware').value

        # State
        self.current_channel = -1

        # Service for switching channels
        self.srv = self.create_service(SetInt32, '~/set_channel', self.switch_channel_callback)
        
        # Subscriber for quick switching
        self.sub = self.create_subscription(Int32, '~/cmd', self.cmd_callback, 10)

        # Init I2C
        try:
            self.bus = smbus2.SMBus(self.bus_id)
            self.get_logger().info(f"TCA9548A initialized on bus {self.bus_id} at 0x{self.address:02X}")
            
            # Set default state
            def_ch = self.get_parameter('default_channel').value
            self._apply_channel(def_ch)
            
        except Exception as e:
            self.get_logger().error(f"Failed to init I2C Mux: {e}")
            self.mock_mode = True

        if self.mock_mode:
            self.get_logger().warn("Running in MOCK mode.")

    def _apply_channel(self, channel):
        """
        Internal method to write to TCA9548A register.
        Channel: 0-7, or -1 to disable all.
        """
        if channel < -1 or channel > 7:
            self.get_logger().error(f"Invalid channel index: {channel}")
            return False

        # Command byte: bit 0 controls CH0, bit 7 controls CH7
        if channel == -1:
            control_byte = 0x00
        else:
            control_byte = 1 << channel

        try:
            if not self.mock_mode:
                self.bus.write_byte(self.address, control_byte)
            
            self.current_channel = channel
            state_str = f"CH{channel}" if channel != -1 else "ALL DISABLED"
            self.get_logger().info(f"I2C Mux switched to: {state_str}")
            return True
        except Exception as e:
            self.get_logger().error(f"I2C Write Error: {e}")
            return False

    def switch_channel_callback(self, request, response):
        success = self._apply_channel(request.data)
        response.success = success
        response.message = f"Channel set to {request.data}" if success else "Failed to set channel"
        return response

    def cmd_callback(self, msg):
        self._apply_channel(msg.data)

    def destroy_node(self):
        # Optional: Disable all channels on exit for safety
        self._apply_channel(-1)
        self.bus.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = I2CMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
