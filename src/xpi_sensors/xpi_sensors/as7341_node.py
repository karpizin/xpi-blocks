#!/usr/bin/env python3
import time
import board
import busio
from adafruit_as7341 import AS7341

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

class AS7341Node(Node):
    def __init__(self):
        super().__init__('as7341_node')
        
        # Parameters
        self.declare_parameter('polling_rate', 10.0)
        self.declare_parameter('gain', 3) # 3 corresponds to 16x gain usually
        self.declare_parameter('integration_time', 29) # Default ~2.78ms * (29+1) = ~83ms

        self.polling_rate = self.get_parameter('polling_rate').value
        self.gain_val = self.get_parameter('gain').value
        self.itime_val = self.get_parameter('integration_time').value

        # Publishers
        self.spectrum_pub = self.create_publisher(Float32MultiArray, '~/spectrum', 10)

        # Hardware Init
        try:
            i2c = board.I2C()  # uses board.SCL and board.SDA
            self.sensor = AS7341(i2c)
            
            # Apply Params (Note: Validating mapping depends on library version, doing raw set if property exists)
            # Adafruit lib exposes .gain and .atime
            try:
                self.sensor.gain = self.gain_val
                self.sensor.atime = self.itime_val
            except Exception as p_err:
                self.get_logger().warn(f"Could not set advanced params: {p_err}")

            self.get_logger().info("AS7341 Spectral Sensor Initialized")
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize AS7341: {e}")
            self.get_logger().error("Ensure I2C is enabled and Adafruit-Blinka is installed.")
            return

        # Timer
        self.timer = self.create_timer(1.0 / self.polling_rate, self.timer_callback)

    def timer_callback(self):
        try:
            # Read all channels
            # The library typically returns them as individual properties
            # We will pack them into an array: F1..F8, Clear, NIR
            
            channels = [
                self.sensor.channel_415nm, # F1
                self.sensor.channel_445nm, # F2
                self.sensor.channel_480nm, # F3
                self.sensor.channel_515nm, # F4
                self.sensor.channel_555nm, # F5
                self.sensor.channel_590nm, # F6
                self.sensor.channel_630nm, # F7
                self.sensor.channel_680nm, # F8
                self.sensor.channel_clear,
                self.sensor.channel_nir
            ]
            
            msg = Float32MultiArray()
            
            # Define Layout
            dim = MultiArrayDimension()
            dim.label = "channels_F1_to_F8_Clear_NIR"
            dim.size = 10
            dim.stride = 10
            msg.layout.dim = [dim]
            
            # Convert to float
            msg.data = [float(x) for x in channels]
            
            self.spectrum_pub.publish(msg)

        except Exception as e:
            self.get_logger().warning(f"Error reading AS7341: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = AS7341Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
