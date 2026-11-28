#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Int32

from xpi_commons.i2c_helper import get_smbus
from xpi_commons.hardware import hardware_unavailable

class I2CMuxNode(Node):
    """
    ROS2 Node for TCA9548A I2C Multiplexer.
    Enables switching between 8 downstream I2C channels.

    Subscribes: ~/cmd (std_msgs/Int32)  channel 0-7, or -1 to disable all
    Publishes:  ~/current_channel (std_msgs/Int32, transient local)  channel actually active
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
        self.bus = None

        # Active channel, latched so late subscribers get the current state
        # (replaces the reply of the former set_channel service)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(Int32, '~/current_channel', latched)

        # Channel selection: 0-7, or -1 to disable all
        self.sub = self.create_subscription(Int32, '~/cmd', self.cmd_callback, 10)

        # Init I2C
        try:
            self.bus = get_smbus(self.bus_id, mock=self.mock_mode)
            self.mock_mode = self.bus.mock_mode
            self.get_logger().info(f"TCA9548A initialized on bus {self.bus_id} at 0x{self.address:02X}")
        except Exception as e:
            if not self.mock_mode:
                hardware_unavailable(self, 'TCA9548A I2C mux', e)
            self.get_logger().error(f"Failed to init I2C Mux: {e}")

        if self.mock_mode:
            self.get_logger().warn("Running in MOCK mode.")

        # Set default state (also publishes the initial ~/current_channel)
        self._apply_channel(self.get_parameter('default_channel').value)

    def _apply_channel(self, channel):
        """
        Internal method to write to TCA9548A register.
        Channel: 0-7, or -1 to disable all.
        """
        if channel < -1 or channel > 7:
            self.get_logger().error(f"Invalid channel index: {channel}")
            self.state_pub.publish(Int32(data=self.current_channel))
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
        finally:
            # Always report the channel that is really active (unchanged after a failure)
            self.state_pub.publish(Int32(data=self.current_channel))

    def cmd_callback(self, msg):
        self._apply_channel(msg.data)

    def destroy_node(self):
        # Optional: Disable all channels on exit for safety
        self._apply_channel(-1)
        if self.bus is not None:
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
