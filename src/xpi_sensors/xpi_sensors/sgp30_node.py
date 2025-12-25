#!/usr/bin/env python3
import time
import board
import busio
import adafruit_sgp30

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32MultiArray

class SGP30Node(Node):
    """
    ROS2 Driver for SGP30 Gas Sensor (TVOC and eCO2) via I2C.
    Highly reliable sensor for indoor air quality.
    """
    def __init__(self):
        super().__init__('sgp30_node')
        
        # Parameters
        self.declare_parameter('polling_rate', 1.0) # 1Hz recommended for SGP30
        self.polling_rate = self.get_parameter('polling_rate').value

        # Publishers
        self.eco2_pub = self.create_publisher(Int32, '~/eco2', 10)
        self.tvoc_pub = self.create_publisher(Int32, '~/tvoc', 10)
        self.raw_pub = self.create_publisher(Float32MultiArray, '~/raw_signals', 10)

        # Hardware Init
        try:
            i2c = board.I2C() 
            self.sgp30 = adafruit_sgp30.Adafruit_SGP30(i2c)
            
            self.get_logger().info("SGP30 initialized. Serial: " + 
                                   str([hex(i) for i in self.sgp30.serial_number]))
            
            # SGP30 needs to be initialized with its baseline if available
            # For now, we just start fresh. It takes ~15 seconds to give valid readings.
            self.sgp30.iaq_init()
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize SGP30: {e}")
            return

        # Timer
        self.timer = self.create_timer(1.0 / self.polling_rate, self.timer_callback)
        self.start_time = time.time()

    def timer_callback(self):
        try:
            eco2, tvoc = self.sgp30.iaq_measure()
            
            # Publish eCO2 (ppm)
            msg_eco2 = Int32()
            msg_eco2.data = int(eco2)
            self.eco2_pub.publish(msg_eco2)
            
            # Publish TVOC (ppb)
            msg_tvoc = Int32()
            msg_tvoc.data = int(tvoc)
            self.tvoc_pub.publish(msg_tvoc)
            
            # Publish Raw H2 and Ethanol for advanced users
            msg_raw = Float32MultiArray()
            msg_raw.data = [float(self.sgp30.raw_H2), float(self.sgp30.raw_ethanol)]
            self.raw_pub.publish(msg_raw)

            elapsed = time.time() - self.start_time
            if elapsed < 15:
                self.get_logger().debug(f"SGP30 Warming up... {int(15-elapsed)}s left")
            else:
                self.get_logger().debug(f"eCO2: {eco2} ppm, TVOC: {tvoc} ppb")

        except Exception as e:
            self.get_logger().warning(f"Error reading SGP30: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SGP30Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
