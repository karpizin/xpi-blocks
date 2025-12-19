#!/usr/bin/env python3
import time
import board
import busio
import adafruit_ccs811

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Int32

class CCS811Node(Node):
    def __init__(self):
        super().__init__('ccs811_node')
        
        # Parameters
        self.declare_parameter('polling_rate', 1.0) # 1Hz is recommended for CCS811 drive modes
        self.polling_rate = self.get_parameter('polling_rate').value

        # Publishers
        self.data_pub = self.create_publisher(Float32MultiArray, '~/data', 10)
        self.eco2_pub = self.create_publisher(Int32, '~/eco2', 10)
        self.tvoc_pub = self.create_publisher(Int32, '~/tvoc', 10)

        # Hardware Init
        try:
            i2c = board.I2C()  # uses board.SCL and board.SDA
            self.sensor = adafruit_ccs811.CCS811(i2c)
            
            # Wait for sensor to be ready
            while not self.sensor.data_ready:
                time.sleep(0.1)
                
            self.get_logger().info("CCS811 Air Quality Sensor Initialized")
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize CCS811: {e}")
            self.get_logger().error("Ensure WAKE pin is grounded and I2C is enabled.")
            return

        # Timer
        self.timer = self.create_timer(1.0 / self.polling_rate, self.timer_callback)

    def timer_callback(self):
        try:
            if self.sensor.data_ready:
                eco2 = self.sensor.eco2
                tvoc = self.sensor.tvoc
                
                # Publish Combined
                msg = Float32MultiArray()
                msg.data = [float(eco2), float(tvoc)]
                
                dim = MultiArrayDimension()
                dim.label = "eCO2_ppm_TVOC_ppb"
                dim.size = 2
                dim.stride = 2
                msg.layout.dim = [dim]
                
                self.data_pub.publish(msg)
                
                # Publish Individual
                self.eco2_pub.publish(Int32(data=eco2))
                self.tvoc_pub.publish(Int32(data=tvoc))
            else:
                # Sensor internal clock stretching or not ready yet
                pass

        except Exception as e:
            self.get_logger().warning(f"Error reading CCS811: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CCS811Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
