#!/usr/bin/env python3
import time
import board
import busio
import adafruit_bme680

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature, RelativeHumidity, FluidPressure
from std_msgs.msg import Float32

class BME680Node(Node):
    """
    ROS2 Node for the BME680 environmental sensor.
    Reads temperature, humidity, pressure, and gas resistance via I2C using Adafruit library.
    """
    def __init__(self):
        super().__init__('bme680_node')
        
        # Parameters
        self.declare_parameter('i2c_address', 0x77) # Default is usually 0x77 for Adafruit, 0x76 for others
        self.declare_parameter('polling_rate', 1.0) 
        self.declare_parameter('frame_id', 'bme680_link')
        self.declare_parameter('sea_level_pressure', 1013.25)
        
        self.polling_rate = self.get_parameter('polling_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.i2c_address = self.get_parameter('i2c_address').value
        
        # Publishers
        self.temp_pub = self.create_publisher(Temperature, '~/temperature', 10)
        self.hum_pub = self.create_publisher(RelativeHumidity, '~/humidity', 10)
        self.press_pub = self.create_publisher(FluidPressure, '~/pressure', 10)
        self.gas_pub = self.create_publisher(Float32, '~/gas_resistance', 10)

        # Hardware Init
        try:
            i2c = board.I2C()  # uses board.SCL and board.SDA
            # Try to initialize with the specified address
            self.sensor = adafruit_bme680.Adafruit_BME680_I2C(i2c, address=self.i2c_address)
            
            # Set sea level pressure if you want altitude (optional, library uses it for altitude calc)
            self.sensor.sea_level_pressure = self.get_parameter('sea_level_pressure').value
            
            self.get_logger().info(f"BME680 Initialized at address 0x{self.i2c_address:02X}")
            
        except ValueError as e:
             self.get_logger().error(f"Failed to init BME680 at 0x{self.i2c_address:02X}. Try changing i2c_address parameter. Error: {e}")
             return
        except Exception as e:
            self.get_logger().error(f"Failed to initialize BME680: {e}")
            return

        # Timer
        self.timer = self.create_timer(1.0 / self.polling_rate, self.timer_callback)

    def timer_callback(self):
        try:
            # Read data
            temp = self.sensor.temperature
            gas = self.sensor.gas
            humidity = self.sensor.relative_humidity
            pressure = self.sensor.pressure
            
            current_time = self.get_clock().now().to_msg()
            
            # Temperature
            temp_msg = Temperature()
            temp_msg.header.stamp = current_time
            temp_msg.header.frame_id = self.frame_id
            temp_msg.temperature = temp
            self.temp_pub.publish(temp_msg)
            
            # Humidity
            hum_msg = RelativeHumidity()
            hum_msg.header.stamp = current_time
            hum_msg.header.frame_id = self.frame_id
            hum_msg.relative_humidity = humidity / 100.0 # Convert % to 0-1
            self.hum_pub.publish(hum_msg)
            
            # Pressure
            press_msg = FluidPressure()
            press_msg.header.stamp = current_time
            press_msg.header.frame_id = self.frame_id
            press_msg.fluid_pressure = pressure * 100.0 # hPa to Pascals
            self.press_pub.publish(press_msg)
            
            # Gas Resistance (Ohms)
            gas_msg = Float32()
            gas_msg.data = float(gas)
            self.gas_pub.publish(gas_msg)
            
            self.get_logger().debug(f'T={temp:.1f}C H={humidity:.1f}% P={pressure:.1f}hPa G={gas}Ohms')

        except Exception as e:
            self.get_logger().warning(f"Error reading BME680: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BME680Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
