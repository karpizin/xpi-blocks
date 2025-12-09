#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Illuminance
import smbus2
import time

class BH1750Node(Node):
    """
    ROS2 Driver for BH1750 Light Sensor via I2C.
    Publishes illuminance in Lux.
    """
    
    # BH1750 Constants
    POWER_DOWN = 0x00
    POWER_ON = 0x01
    RESET = 0x07
    
    # Measurement Modes
    CONTINUOUS_HIGH_RES_MODE = 0x10
    CONTINUOUS_HIGH_RES_MODE_2 = 0x11
    CONTINUOUS_LOW_RES_MODE = 0x13
    ONE_TIME_HIGH_RES_MODE = 0x20
    ONE_TIME_HIGH_RES_MODE_2 = 0x21
    ONE_TIME_LOW_RES_MODE = 0x23

    def __init__(self):
        super().__init__('bh1750_node')
        
        # Parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x23) # Default is 0x23, secondary is 0x5C
        self.declare_parameter('publish_rate', 1.0) # Hz
        self.declare_parameter('frame_id', 'bh1750_link')
        self.declare_parameter('mode', 'CONTINUOUS_HIGH_RES_MODE') # Mode name
        
        self.i2c_bus_id = self.get_parameter('i2c_bus').value
        self.device_address = self.get_parameter('i2c_address').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        mode_str = self.get_parameter('mode').value

        # Map mode string to constant
        if hasattr(self, mode_str):
            self.mode = getattr(self, mode_str)
        else:
            self.get_logger().warn(f"Unknown mode '{mode_str}', defaulting to CONTINUOUS_HIGH_RES_MODE")
            self.mode = self.CONTINUOUS_HIGH_RES_MODE

        # Publisher
        self.pub = self.create_publisher(Illuminance, 'illuminance', 10)
        
        # Initialize I2C
        try:
            self.bus = smbus2.SMBus(self.i2c_bus_id)
            self.get_logger().info(f"Connected to I2C bus {self.i2c_bus_id}")
            self.init_sensor()
        except Exception as e:
            self.get_logger().error(f"Failed to connect to I2C bus: {e}")
            return

        # Timer
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        self.get_logger().info(f"BH1750 Node started. Addr: 0x{self.device_address:02X}, Rate: {self.publish_rate}Hz")

    def init_sensor(self):
        try:
            self.bus.write_byte(self.device_address, self.POWER_ON)
            self.bus.write_byte(self.device_address, self.RESET)
            self.bus.write_byte(self.device_address, self.mode)
            time.sleep(0.2) # Wait for mode to apply
        except Exception as e:
            self.get_logger().error(f"Failed to initialize BH1750: {e}")

    def read_light(self):
        try:
            # Read 2 bytes from I2C
            data = self.bus.read_i2c_block_data(self.device_address, self.mode, 2)
            
            # Convert 2 bytes to integer
            # Result = (High Byte * 256 + Low Byte) / 1.2
            val = (data[0] << 8) | data[1]
            lux = val / 1.2
            return lux
        except Exception as e:
            self.get_logger().warn(f"I2C Read Error: {e}")
            return None

    def timer_callback(self):
        lux = self.read_light()
        
        if lux is not None:
            msg = Illuminance()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.illuminance = lux
            msg.variance = 0.0 # Variance unknown
            
            self.pub.publish(msg)
            # self.get_logger().debug(f"Light: {lux:.2f} lux")

    def destroy_node(self):
        if hasattr(self, 'bus'):
            self.bus.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BH1750Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
