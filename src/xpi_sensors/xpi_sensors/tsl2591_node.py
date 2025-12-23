#!/usr/bin/env python3
import time
import board
import busio
import adafruit_tsl2591

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Illuminance
from std_msgs.msg import Float32

class TSL2591Node(Node):
    """
    ROS2 Node for the TSL2591 High Dynamic Range Light Sensor.
    Publishes Illuminance (Lux) and Infrared level.
    """
    def __init__(self):
        super().__init__('tsl2591_node')
        
        # Parameters
        self.declare_parameter('polling_rate', 2.0) # Hz
        self.declare_parameter('gain', 'MEDIUM') # LOW, MEDIUM, HIGH, MAX
        self.declare_parameter('integration_time', '100MS') # 100MS, 200MS, 300MS, 400MS, 500MS, 600MS
        self.declare_parameter('frame_id', 'tsl2591_link')
        
        self.polling_rate = self.get_parameter('polling_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        gain_str = self.get_parameter('gain').value.upper()
        int_time_str = self.get_parameter('integration_time').value.upper()

        # Publishers
        self.lux_pub = self.create_publisher(Illuminance, '~/illuminance', 10)
        self.ir_pub = self.create_publisher(Illuminance, '~/infrared', 10) # Using Illuminance msg for IR intensity (W/m^2 approx or raw)
        # Note: TSL2591 IR output is raw counts, not directly W/m^2 without cal, but we use Illuminance type for consistency or Float32. 
        # Let's use Float32 for IR raw counts to avoid confusion with Lux.
        self.ir_raw_pub = self.create_publisher(Float32, '~/infrared_raw', 10)

        # Hardware Init
        try:
            i2c = board.I2C()
            self.sensor = adafruit_tsl2591.TSL2591(i2c)
            
            # Set Gain
            if gain_str == 'LOW': self.sensor.gain = adafruit_tsl2591.GAIN_LOW
            elif gain_str == 'MEDIUM': self.sensor.gain = adafruit_tsl2591.GAIN_MED
            elif gain_str == 'HIGH': self.sensor.gain = adafruit_tsl2591.GAIN_HIGH
            elif gain_str == 'MAX': self.sensor.gain = adafruit_tsl2591.GAIN_MAX
            else: self.get_logger().warn(f"Unknown gain '{gain_str}', using MEDIUM.")

            # Set Integration Time
            if int_time_str == '100MS': self.sensor.integration_time = adafruit_tsl2591.INTEGRATIONTIME_100MS
            elif int_time_str == '200MS': self.sensor.integration_time = adafruit_tsl2591.INTEGRATIONTIME_200MS
            elif int_time_str == '300MS': self.sensor.integration_time = adafruit_tsl2591.INTEGRATIONTIME_300MS
            elif int_time_str == '400MS': self.sensor.integration_time = adafruit_tsl2591.INTEGRATIONTIME_400MS
            elif int_time_str == '500MS': self.sensor.integration_time = adafruit_tsl2591.INTEGRATIONTIME_500MS
            elif int_time_str == '600MS': self.sensor.integration_time = adafruit_tsl2591.INTEGRATIONTIME_600MS
            else: self.get_logger().warn(f"Unknown integration time '{int_time_str}', using 100MS.")

            self.get_logger().info(f"TSL2591 Initialized. Gain: {gain_str}, Int. Time: {int_time_str}")

        except Exception as e:
            self.get_logger().error(f"Failed to initialize TSL2591: {e}")
            self.sensor = None
            return

        self.timer = self.create_timer(1.0 / self.polling_rate, self.timer_callback)

    def timer_callback(self):
        if not self.sensor:
            return
            
        try:
            # Read data
            lux = self.sensor.lux
            infrared = self.sensor.infrared
            visible = self.sensor.visible
            
            current_time = self.get_clock().now().to_msg()
            
            # Lux
            lux_msg = Illuminance()
            lux_msg.header.stamp = current_time
            lux_msg.header.frame_id = self.frame_id
            lux_msg.illuminance = float(lux) if lux is not None else 0.0
            self.lux_pub.publish(lux_msg)
            
            # IR (Raw)
            ir_msg = Float32()
            ir_msg.data = float(infrared)
            self.ir_raw_pub.publish(ir_msg)
            
            # Log
            self.get_logger().debug(f"Lux: {lux:.2f}, IR: {infrared}, Visible: {visible}")

        except Exception as e:
            self.get_logger().warn(f"Error reading TSL2591: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TSL2591Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
