#!/usr/bin/env python3
import time
import os
import yaml
import math
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
        self.emotions = self.load_emotions()
        
        # State: [lid_t, lid_b, pupil_s, y_off, x_shift, brow_t, brow_y, mouth_c, mouth_o]
        self.current_params = np.array(self.emotions.get("NEUTRAL", [0]*9), dtype=float)
        self.target_params = np.array(self.emotions.get("NEUTRAL", [0]*9), dtype=float)
        self.transition_speed = 0.15
        self.current_emotion_name = "NEUTRAL"
        
        # Animation State
        self.start_time = time.time()
        self.last_blink_time = time.time()
        self.blink_duration = 0.15 # seconds
        self.is_blinking = False
        
        # Publishers/Subscribers
        self.image_pub = self.create_publisher(ImageMsg, '~/eyes_image', 10)
        self.sub = self.create_subscription(String, '~/set_expression', self.cb_set_expression, 10)
        
        # Timer (30 FPS for smoother animation)
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)
        self.get_logger().info(f"Animated Expression Engine Ready")

    def load_emotions(self):
        try:
            pkg_path = get_package_share_directory('xpi_hci')
            yaml_path = os.path.join(pkg_path, 'config', 'emotions.yaml')
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
                # Add aliases for animated versions if not in YAML
                if "IDLE_ANIMATED" not in data: data["IDLE_ANIMATED"] = data["NEUTRAL"]
                return data
        except Exception as e:
            return {"NEUTRAL": [0, 0, 1.0, 0, 0, 0, 0, 0, 0]}

    def cb_set_expression(self, msg):
        name = msg.data.upper()
        if name in self.emotions:
            self.target_params = np.array(self.emotions[name], dtype=float)
            self.current_emotion_name = name
        else:
            self.get_logger().warn(f"Emotion {name} not found")

    def timer_callback(self):
        t = time.time() - self.start_time
        
        # 1. Base Transition (Lerp to target)
        self.current_params += (self.target_params - self.current_params) * self.transition_speed
        
        # 2. Apply Internal Animations based on emotion name
        anim_params = self.current_params.copy()
        
        # A. Auto-Blink Logic (Global for most emotions)
        if not self.is_blinking and (time.time() - self.last_blink_time) > 4.0:
            if np.random.random() > 0.95: # Random chance to blink
                self.is_blinking = True
                self.blink_start = time.time()
        
        if self.is_blinking:
            dt = time.time() - self.blink_start
            if dt < self.blink_duration:
                anim_params[0] = 85 # Force lids closed
                anim_params[1] = 85
            else:
                self.is_blinking = False
                self.last_blink_time = time.time()

        # B. Specific Emotion Animations
        if "ANIMATED" in self.current_emotion_name or self.current_emotion_name in ["LAUGHING", "ECSTATIC", "THINKING"]:
            
            if "IDLE" in self.current_emotion_name or "NEUTRAL" in self.current_emotion_name:
                # Breathing (slow sine on Y)
                anim_params[3] += math.sin(t * 1.5) * 3
                # Subtle pupil pulse
                anim_params[2] *= (1.0 + math.sin(t * 0.5) * 0.05)
                
            elif "LAUGHING" in self.current_emotion_name:
                # High frequency shake
                anim_params[3] += math.sin(t * 20) * 5
                # Mouth pulse
                anim_params[8] += math.sin(t * 15) * 10
                
            elif "ECSTATIC" in self.current_emotion_name:
                # Pupil "shining" (fast scale pulse)
                anim_params[2] *= (1.0 + math.sin(t * 10) * 0.15)
                # Bouncing
                anim_params[3] += abs(math.sin(t * 8)) * -10
                
            elif "THINKING" in self.current_emotion_name:
                # Eyes moving left-right
                anim_params[4] += math.sin(t * 2) * 20

        # 3. Render and Publish
        img = self.render_face(anim_params)
        ros_msg = self.bridge.cv2_to_imgmsg(np.array(img), encoding="rgb8")
        self.image_pub.publish(ros_msg)

    def render_face(self, p):
        # p: [lid_t, lid_b, pupil_s, y_off, x_shift, brow_t, brow_y, mouth_c, mouth_o]
        img = Image.new('RGB', self.canvas_size, color=(0, 0, 0))
        draw = ImageDraw.Draw(img)
        
        eye_w, eye_h = 60, 80
        eye_spacing = 100
        center_x, center_y = self.canvas_size[0]//2, self.canvas_size[1]//2
        
        # 1. Eyes
        for side in [-1, 1]:
            cx = center_x + (side * eye_spacing//2) + p[4]
            cy = center_y + p[3] - 20
            r = int(eye_w * p[2] / 2)
            draw.ellipse([cx-r, cy-r, cx+r, cy+r], fill=(255, 255, 255))
            lt, lb = p[0], p[1]
            draw.chord([cx-eye_w, cy-eye_h-10, cx+eye_w, cy+eye_h-10], start=180-lt, end=360+lt, fill=(0,0,0))
            draw.chord([cx-eye_w, cy-eye_h+10, cx+eye_w, cy+eye_h+10], start=0-lb, end=180+lb, fill=(0,0,0))

        # 2. Brows
        for side in [-1, 1]:
            bx = center_x + (side * eye_spacing//2) + p[4]
            by = center_y + p[3] - 70 + p[6]
            tilt = side * p[5]
            brow_len = 50
            x1, y1 = bx - brow_len//2, by - tilt//2
            x2, y2 = bx + brow_len//2, by + tilt//2
            draw.line([x1, y1, x2, y2], fill=(255, 255, 255), width=8)

        # 3. Mouth
        mx, my = center_x + p[4]*0.5, center_y + 60 + p[3]*0.5
        m_width, m_curve, m_open = 80, p[7], p[8]
        if m_open > 5:
            draw.ellipse([mx-m_width//2, my-m_open//2, mx+m_width//2, my+m_open//2], fill=(255, 255, 255))
        else:
            bbox = [mx-m_width//2, my-abs(m_curve)-10, mx+m_width//2, my+abs(m_curve)+10]
            if m_curve >= 0: draw.arc(bbox, start=0, end=180, fill=(255, 255, 255), width=6)
            else: draw.arc(bbox, start=180, end=360, fill=(255, 255, 255), width=6)
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
