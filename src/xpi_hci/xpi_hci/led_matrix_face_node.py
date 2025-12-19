#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ImageMsg
from std_msgs.msg import UInt8MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np

class LEDMatrixFaceNode(Node):
    def __init__(self):
        super().__init__('led_matrix_face_node')
        self.bridge = CvBridge()
        
        # Parameters
        self.declare_parameter('width', 16)
        self.declare_parameter('height', 16)
        self.w = self.get_parameter('width').value
        self.h = self.get_parameter('height').value
        
        # Sub/Pub
        self.sub = self.create_subscription(ImageMsg, '/expression_engine_node/eyes_image', self.image_callback, 10)
        self.pub = self.create_publisher(UInt8MultiArray, '~/matrix_raw', 10)
        
        self.get_logger().info(f"LED Matrix Adapter Ready ({self.w}x{self.h})")

    def image_callback(self, msg):
        # 1. Convert ROS Image to OpenCV
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        
        # 2. Resize to Matrix size
        small_img = cv2.resize(cv_img, (self.w, self.h), interpolation=cv2.INTER_AREA)
        
        # 3. Convert to Grayscale or Bitmask (since most matrixes are monochrome or RGB)
        # Assuming monochrome for this generic block
        gray = cv2.cvtColor(small_img, cv2.COLOR_BGR2GRAY)
        
        # 4. Pack into UInt8MultiArray
        out_msg = UInt8MultiArray()
        out_msg.data = gray.flatten().tolist()
        self.pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LEDMatrixFaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
