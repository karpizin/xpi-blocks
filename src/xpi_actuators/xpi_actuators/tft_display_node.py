#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from PIL import Image as PILImage
from PIL import ImageDraw, ImageFont
import numpy as np

import board
import busio
import digitalio
from adafruit_rgb_display import st7789, st7735

class TFTDisplayNode(Node):
    """
    ROS2 Node for ST7789 / ST7735 SPI TFT Displays.
    - Subscribes to Image topics to show graphics.
    - Subscribes to String topics for quick status text.
    """
    def __init__(self):
        super().__init__('tft_display_node')

        # 1. Parameters
        self.declare_parameter('display_type', 'ST7789') # ST7789 or ST7735
        self.declare_parameter('width', 240)
        self.declare_parameter('height', 240)
        self.declare_parameter('rotation', 0)
        self.declare_parameter('baudrate', 24000000) # 24MHz
        
        # GPIO Pins (BCM)
        self.declare_parameter('cs_pin', 8)   # CE0
        self.declare_parameter('dc_pin', 25)
        self.declare_parameter('rst_pin', 27)
        self.declare_parameter('bl_pin', 24)  # Backlight

        self.display_type = self.get_parameter('display_type').value.upper()
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.rotation = self.get_parameter('rotation').value

        # 2. Hardware Setup
        try:
            spi = board.SPI()
            cs = digitalio.DigitalInOut(getattr(board, f"D{self.get_parameter('cs_pin').value}"))
            dc = digitalio.DigitalInOut(getattr(board, f"D{self.get_parameter('dc_pin').value}"))
            rst = digitalio.DigitalInOut(getattr(board, f"D{self.get_parameter('rst_pin').value}"))
            
            # Backlight
            bl = digitalio.DigitalInOut(getattr(board, f"D{self.get_parameter('bl_pin').value}"))
            bl.direction = digitalio.Direction.OUTPUT
            bl.value = True # Turn on

            if self.display_type == 'ST7789':
                self.display = st7789.ST7789(
                    spi, cs=cs, dc=dc, rst=rst, 
                    width=self.width, height=self.height, 
                    rotation=self.rotation, baudrate=self.get_parameter('baudrate').value
                )
            else: # ST7735
                self.display = st7735.ST7735R(
                    spi, cs=cs, dc=dc, rst=rst, 
                    width=self.width, height=self.height, 
                    rotation=self.rotation, baudrate=self.get_parameter('baudrate').value
                )
            
            self.get_logger().info(f"Initialized {self.display_type} ({self.width}x{self.height})")
        except Exception as e:
            self.get_logger().error(f"Failed to init display: {e}")
            self.display = None

        # 3. ROS Interfaces
        self.bridge = CvBridge()
        self.create_subscription(Image, '~/image_in', self._image_callback, 10)
        self.create_subscription(String, '~/text_in', self._text_callback, 10)

        # Buffer for text rendering
        self.font = ImageFont.load_default()
        self.get_logger().info("TFT Display Node Ready.")

    def _image_callback(self, msg):
        if not self.display: return
        try:
            # Convert ROS Image to OpenCV
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            # Resize if needed
            if cv_img.shape[0] != self.height or cv_img.shape[1] != self.width:
                cv_img = cv2.resize(cv_img, (self.width, self.height))
            
            # Convert to PIL and show
            pil_img = PILImage.fromarray(cv_img)
            self.display.image(pil_img)
        except Exception as e:
            self.get_logger().warn(f"Display Error: {e}")

    def _text_callback(self, msg):
        if not self.display: return
        try:
            # Create black image
            img = PILImage.new("RGB", (self.width, self.height), (0, 0, 0))
            draw = ImageDraw.Draw(img)
            
            # Simple multiline text wrapping
            y = 10
            for line in msg.data.split('\n'):
                draw.text((10, y), line, font=self.font, fill=(255, 255, 255))
                y += 20
            
            self.display.image(img)
        except Exception as e:
            self.get_logger().warn(f"Text Display Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TFTDisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
