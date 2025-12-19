#!/usr/bin/env python3
import time
import os
import yaml
import numpy as np
from PIL import Image, ImageDraw
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String
from sensor_msgs.msg import Image as ImageMsg
from cv_bridge import CvBridge

class ExpressionEngineNode(Node):
    def __init__(self):
        super().__init__('expression_engine_node')
        
        self.canvas_size = (256, 256)
        self.bridge = CvBridge()
        
        # Load Emotions from YAML
        self.emotions = self.load_emotions()
        
        # State: [lid_t, lid_b, pupil_s, y_off, x_shift, brow_t, brow_y, mouth_c, mouth_o]
        self.current_params = np.array(self.emotions.get("NEUTRAL", [0]*9), dtype=float)
        self.target_params = np.array(self.emotions.get("NEUTRAL", [0]*9), dtype=float)
        self.transition_speed = 0.2
        
        # Publishers/Subscribers
        self.image_pub = self.create_publisher(ImageMsg, '~/eyes_image', 10)
        self.sub = self.create_subscription(String, '~/set_expression', self.cb_set_expression, 10)
        
        # Timer (25 FPS)
        self.timer = self.create_timer(1.0/25.0, self.timer_callback)
        self.get_logger().info(f"Expression Engine Started with {len(self.emotions)} emotions")

    def load_emotions(self):
        try:
            # Try to find the config in the install/share directory
            pkg_path = get_package_share_directory('xpi_hci')
            yaml_path = os.path.join(pkg_path, 'config', 'emotions.yaml')
            with open(yaml_path, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            self.get_logger().warn(f"Could not load emotions.yaml: {e}. Using fallback.")
            return {"NEUTRAL": [0, 0, 1.0, 0, 0, 0, 0, 0, 0]}

    def cb_set_expression(self, msg):
        name = msg.data.upper()
        if name in self.emotions:
            self.target_params = np.array(self.emotions[name], dtype=float)
            # Special case for WINK Logic if needed
        else:
            self.get_logger().warn(f"Unknown emotion: {name}")

    def timer_callback(self):
        # Smooth Transition
        self.current_params += (self.target_params - self.current_params) * self.transition_speed
        
        # Render
        img = self.render_face(self.current_params)
        
        # Publish
        ros_msg = self.bridge.cv2_to_imgmsg(np.array(img), encoding="rgb8")
        self.image_pub.publish(ros_msg)

    def render_face(self, p):
        # p: [lid_t, lid_b, pupil_s, y_off, x_shift, brow_t, brow_y, mouth_c, mouth_o]
        img = Image.new('RGB', self.canvas_size, color=(0, 0, 0))
        draw = ImageDraw.Draw(img)
        
        eye_w, eye_h = 60, 80
        eye_spacing = 100
        center_x, center_y = self.canvas_size[0]//2, self.canvas_size[1]//2
        
        # 1. Draw EYES
        for side in [-1, 1]:
            cx = center_x + (side * eye_spacing//2) + p[4]
            cy = center_y + p[3] - 20 # Eyes a bit higher than center
            
            # Pupil
            r = int(eye_w * p[2] / 2)
            draw.ellipse([cx-r, cy-r, cx+r, cy+r], fill=(255, 255, 255))
            
            # Lids
            lt, lb = p[0], p[1]
            draw.chord([cx-eye_w, cy-eye_h-10, cx+eye_w, cy+eye_h-10], start=180-lt, end=360+lt, fill=(0,0,0))
            draw.chord([cx-eye_w, cy-eye_h+10, cx+eye_w, cy+eye_h+10], start=0-lb, end=180+lb, fill=(0,0,0))

        # 2. Draw BROWS
        for side in [-1, 1]:
            bx = center_x + (side * eye_spacing//2) + p[4]
            by = center_y + p[3] - 70 + p[6] # Above eyes
            tilt = side * p[5]
            
            # Draw brow as a thick line/rotated rectangle
            brow_len = 50
            x1 = bx - brow_len//2
            y1 = by - tilt//2
            x2 = bx + brow_len//2
            y2 = by + tilt//2
            draw.line([x1, y1, x2, y2], fill=(255, 255, 255), width=8)

        # 3. Draw MOUTH
        mx = center_x + p[4]*0.5
        my = center_y + 60 + p[3]*0.5
        m_width = 80
        m_curve = p[7]
        m_open = p[8]
        
        if m_open > 5:
            # Open mouth (ellipse)
            draw.ellipse([mx-m_width//2, my-m_open//2, mx+m_width//2, my+m_open//2], fill=(255, 255, 255))
        else:
            # Closed mouth (arc/smile)
            # If curve > 0: smile, < 0: frown
            bbox = [mx-m_width//2, my-abs(m_curve)-10, mx+m_width//2, my+abs(m_curve)+10]
            if m_curve >= 0:
                draw.arc(bbox, start=0, end=180, fill=(255, 255, 255), width=6)
            else:
                draw.arc(bbox, start=180, end=360, fill=(255, 255, 255), width=6)

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