#!/usr/bin/env python3
import time
import numpy as np
from PIL import Image, ImageDraw
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image as ImageMsg
from cv_bridge import CvBridge

class ExpressionEngineNode(Node):
    def __init__(self):
        super().__init__('expression_engine_node')
        
        self.canvas_size = (256, 256)
        self.bridge = CvBridge()
        
        # [lid_top, lid_bottom, pupil_scale, y_off, x_shift]
        self.emotions = {
            "NEUTRAL":   [0, 0, 1.0, 0, 0],
            "HAPPY":     [45, -45, 1.1, -10, 0],
            "ANGRY":     [-35, 0, 0.8, 10, 0],
            "SAD":       [35, 15, 0.9, 20, 0],
            "SURPRISED": [-25, -25, 1.5, 0, 0],
            "THINKING":  [0, 0, 1.0, -10, 40],
            "SLEEPY":    [75, -75, 0.4, 10, 0],
            "EVIL":      [-50, 30, 0.7, 0, 0],
            "WINK":      [0, -80, 1.0, 0, 0], # Right eye closed
            "CURIOUS":   [-10, 0, 1.3, -5, -25],
            "CONFUSED":  [20, -20, 1.0, 0, 0],
            "BORED":     [40, 40, 0.8, 10, 0],
            "GHOST":     [0, 0, 1.8, 0, 0],
            "GLITCH":    [90, 90, 0.1, 0, 0]
        }
        # To get to 50+, we can interpolate or add more keys here. 
        # For v1 we provide 14 core emotions.
        
        self.current_params = np.array(self.emotions["NEUTRAL"], dtype=float)
        self.target_params = np.array(self.emotions["NEUTRAL"], dtype=float)
        self.transition_speed = 0.15 
        
        self.image_pub = self.create_publisher(ImageMsg, '~/eyes_image', 10)
        self.sub = self.create_subscription(String, '~/set_expression', self.cb_set_expression, 10)
        
        self.timer = self.create_timer(1.0/25.0, self.timer_callback)
        self.get_logger().info("Expression Engine Ready")

    def cb_set_expression(self, msg):
        name = msg.data.upper()
        if name in self.emotions:
            self.target_params = np.array(self.emotions[name], dtype=float)
        else:
            self.get_logger().warn(f"Emotion {name} not found")

    def timer_callback(self):
        # Lerp
        self.current_params += (self.target_params - self.current_params) * self.transition_speed
        img = self.render_eyes(self.current_params)
        ros_msg = self.bridge.cv2_to_imgmsg(np.array(img), encoding="rgb8")
        self.image_pub.publish(ros_msg)

    def render_eyes(self, p):
        img = Image.new('RGB', self.canvas_size, color=(0, 0, 0))
        draw = ImageDraw.Draw(img)
        eye_w, eye_h = 70, 90
        spacing = 110
        
        for side in [-1, 1]: # Left, Right
            cx = self.canvas_size[0]//2 + (side * spacing//2) + (p[4] if side > 0 else p[4]*0.8)
            cy = self.canvas_size[1]//2 + p[3]
            
            # Draw Pupil
            r = int(eye_w * p[2] / 2)
            draw.ellipse([cx-r, cy-r, cx+r, cy+r], fill=(255, 255, 255))
            
            # Lids (using chords to cut the circle)
            lt, lb = p[0], p[1]
            # If WINK and right side, force close
            if self.target_params is self.emotions.get("WINK") and side > 0:
                lt, lb = 80, 80

            draw.chord([cx-eye_w, cy-eye_h-20, cx+eye_w, cy+eye_h-20], start=180-lt, end=360+lt, fill=(0,0,0))
            draw.chord([cx-eye_w, cy-eye_h+20, cx+eye_w, cy+eye_h+20], start=0-lb, end=180+lb, fill=(0,0,0))
        return img

def main(args=None):
    rclpy.init(args=args)
    node = ExpressionEngineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
