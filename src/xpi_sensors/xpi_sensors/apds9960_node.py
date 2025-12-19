#!/usr/bin/env python3
import time
from apds9960.const import *
from apds9960 import APDS9960
import smbus2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, ColorRGBA
from sensor_msgs.msg import Range

class APDS9960Node(Node):
    def __init__(self):
        super().__init__('apds9960_node')
        
        # Parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('polling_rate', 10.0) # Hz
        self.declare_parameter('enable_color', True)
        self.declare_parameter('enable_prox', True)
        self.declare_parameter('enable_gesture', True)

        self.i2c_bus_id = self.get_parameter('i2c_bus').value
        self.polling_rate = self.get_parameter('polling_rate').value
        self.enable_color = self.get_parameter('enable_color').value
        self.enable_prox = self.get_parameter('enable_prox').value
        self.enable_gesture = self.get_parameter('enable_gesture').value

        # Publishers
        if self.enable_color:
            self.color_pub = self.create_publisher(ColorRGBA, '~/color_raw', 10)
        
        if self.enable_prox:
            self.prox_pub = self.create_publisher(Range, '~/proximity', 10)
            
        if self.enable_gesture:
            self.gesture_pub = self.create_publisher(String, '~/gesture', 10)

        # Hardware Init
        try:
            self.bus = smbus2.SMBus(self.i2c_bus_id)
            self.apds = APDS9960(self.bus)
            
            # Reset and Enable
            self.apds.resetGestureParameters()
            self.apds.enablePower()
            
            if self.enable_color:
                self.apds.enableLightSensor()
                self.get_logger().info("Color sensor enabled")
                
            if self.enable_prox:
                self.apds.enableProximitySensor()
                self.get_logger().info("Proximity sensor enabled")
                
            if self.enable_gesture:
                self.apds.enableGestureSensor()
                self.get_logger().info("Gesture sensor enabled")

            self.get_logger().info(f"APDS-9960 Initialized on Bus {self.i2c_bus_id}")

        except Exception as e:
            self.get_logger().error(f"Failed to initialize APDS-9960: {e}")
            return

        # Timer
        self.timer = self.create_timer(1.0 / self.polling_rate, self.timer_callback)

    def timer_callback(self):
        try:
            # 1. Read Color
            if self.enable_color:
                r = self.apds.readRedLight()
                g = self.apds.readGreenLight()
                b = self.apds.readBlueLight()
                c = self.apds.readAmbientLight()
                
                msg = ColorRGBA()
                msg.r = float(r)
                msg.g = float(g)
                msg.b = float(b)
                msg.a = float(c) # Use alpha channel for Clear/Ambient
                self.color_pub.publish(msg)

            # 2. Read Proximity
            if self.enable_prox:
                prox = self.apds.readProximity()
                # Publish as Range
                range_msg = Range()
                range_msg.header.stamp = self.get_clock().now().to_msg()
                range_msg.header.frame_id = "apds9960_frame"
                range_msg.radiation_type = Range.INFRARED
                range_msg.min_range = 0.01
                range_msg.max_range = 0.20
                # Map 0-255 to meters (approximate, inverse relationship usually)
                # But here we just publish raw for simplicity or 0 if far, >0 if near
                range_msg.range = float(prox) / 255.0 * 0.20 
                self.prox_pub.publish(range_msg)

            # 3. Read Gesture
            if self.enable_gesture:
                if self.apds.isGestureAvailable():
                    motion = self.apds.readGesture()
                    gesture_name = self.decode_gesture(motion)
                    if gesture_name:
                        self.get_logger().info(f"Gesture Detected: {gesture_name}")
                        self.gesture_pub.publish(String(data=gesture_name))

        except Exception as e:
            self.get_logger().warning(f"Error reading APDS-9960: {e}")

    def decode_gesture(self, gesture_code):
        if gesture_code == APDS9960_DIR_UP:
            return "UP"
        elif gesture_code == APDS9960_DIR_DOWN:
            return "DOWN"
        elif gesture_code == APDS9960_DIR_LEFT:
            return "LEFT"
        elif gesture_code == APDS9960_DIR_RIGHT:
            return "RIGHT"
        elif gesture_code == APDS9960_DIR_NEAR:
            return "NEAR"
        elif gesture_code == APDS9960_DIR_FAR:
            return "FAR"
        return None

def main(args=None):
    rclpy.init(args=args)
    node = APDS9960Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
