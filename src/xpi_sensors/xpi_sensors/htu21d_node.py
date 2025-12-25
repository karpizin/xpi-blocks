#!/usr/bin/env python3
import time
import board
import busio
import adafruit_htu21d

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature, RelativeHumidity

class HTU21DNode(Node):
    """
    ROS2 Driver for HTU21D and SHT20 Temperature & Humidity Sensors via I2C.
    Highly popular and reliable environment sensors.
    """
    def __init__(self):
        super().__init__('htu21d_node')
        
        # Parameters
        self.declare_parameter('polling_rate', 1.0) # Hz
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('frame_id', 'htu21d_link')

        self.polling_rate = self.get_parameter('polling_rate').value
        self.i2c_bus_id = self.get_parameter('i2c_bus').value
        self.frame_id = self.get_parameter('frame_id').value

        # Publishers
        self.temp_pub = self.create_publisher(Temperature, '~/temperature', 10)
        self.hum_pub = self.create_publisher(RelativeHumidity, '~/humidity', 10)

        # Hardware Init
        try:
            # Note: adafruit_htu21d typically uses default I2C (board.I2C())
            # For multi-bus support, we use busio.I2C
            i2c = board.I2C() 
            self.sensor = adafruit_htu21d.HTU21D(i2c)
            self.get_logger().info(f"HTU21D/SHT20 initialized on bus {self.i2c_bus_id}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize HTU21D: {e}")
            return

        # Timer
        self.timer = self.create_timer(1.0 / self.polling_rate, self.timer_callback)

    def timer_callback(self):
        try:
            temp = self.sensor.temperature
            hum = self.sensor.relative_humidity
            
            # Publish Temperature
            t_msg = Temperature()
            t_msg.header.stamp = self.get_clock().now().to_msg()
            t_msg.header.frame_id = self.frame_id
            t_msg.temperature = float(temp)
            t_msg.variance = 0.0
            self.temp_pub.publish(t_msg)
            
            # Publish Humidity
            h_msg = RelativeHumidity()
            h_msg.header.stamp = self.get_clock().now().to_msg()
            h_msg.header.frame_id = self.frame_id
            h_msg.relative_humidity = float(hum) / 100.0 # Standard: 0.0 to 1.0
            h_msg.variance = 0.0
            self.hum_pub.publish(h_msg)

            # self.get_logger().debug(f"Temp: {temp:.1f}C, Hum: {hum:.1f}%")

        except Exception as e:
            self.get_logger().warning(f"Error reading HTU21D: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = HTU21DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
