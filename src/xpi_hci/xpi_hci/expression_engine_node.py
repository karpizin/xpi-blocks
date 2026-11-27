#!/usr/bin/env python3
import os
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image as ImageMsg
from std_msgs.msg import String
from cv_bridge import CvBridge

from .expression_model import (
    BROW_TILT_LEFT,
    BROW_TILT_RIGHT,
    LID_BOTTOM_LEFT,
    LID_BOTTOM_RIGHT,
    LID_TOP_LEFT,
    LID_TOP_RIGHT,
    PUPIL_SCALE_LEFT,
    PUPIL_SCALE_RIGHT,
    X_SHIFT,
    Y_OFFSET,
    MOUTH_OPEN,
    load_emotions_from_yaml,
    normalize_expression_spec,
    render_face_image,
)

class ExpressionEngineNode(Node):
    def __init__(self):
        super().__init__('expression_engine_node')
        
        self.canvas_size = (256, 256)
        self.bridge = CvBridge()
        self.emotions = self.load_emotions()
        
        self.current_params = np.array(self.emotions.get("NEUTRAL", [0]*9), dtype=float)
        self.target_params = np.array(self.emotions.get("NEUTRAL", [0]*9), dtype=float)
        self.declare_parameter('transition_speed', 0.15)
        self.transition_speed = float(self.get_parameter('transition_speed').value)
        self.current_emotion_name = "NEUTRAL"
        
        # Animation State
        self.start_time = time.time()
        self.last_blink_time = time.time()
        self.blink_duration = 0.15 # seconds
        self.is_blinking = False
        self.blink_start = 0.0
        
        # Publishers/Subscribers
        self.image_pub = self.create_publisher(ImageMsg, '~/eyes_image', 10)
        self.sub = self.create_subscription(String, '~/set_expression', self.cb_set_expression, 10)
        
        # Timer (30 FPS for smoother animation)
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)
        self.get_logger().info("Animated Expression Engine Ready")

    def load_emotions(self):
        try:
            pkg_path = get_package_share_directory('xpi_hci')
            yaml_path = os.path.join(pkg_path, 'config', 'emotions.yaml')
            data = load_emotions_from_yaml(yaml_path)
            self.get_logger().info(f"Loaded {len(data)} facial expressions from {yaml_path}")
            return data
        except Exception as exc:
            self.get_logger().warning(f"Failed to load emotions.yaml, falling back to NEUTRAL only: {exc}")
            return {"NEUTRAL": normalize_expression_spec([5, 5, 1.0, 0, 0, 0, 0, 0, 0])}

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
                anim_params[LID_TOP_LEFT] = 85
                anim_params[LID_BOTTOM_LEFT] = 85
                anim_params[LID_TOP_RIGHT] = 85
                anim_params[LID_BOTTOM_RIGHT] = 85
            else:
                self.is_blinking = False
                self.last_blink_time = time.time()

        # B. Specific Emotion Animations
        if "ANIMATED" in self.current_emotion_name or self.current_emotion_name in ["LAUGHING", "ECSTATIC", "THINKING", "WINK", "WINK_RIGHT"]:
            
            if "IDLE" in self.current_emotion_name or "NEUTRAL" in self.current_emotion_name:
                # Breathing (slow sine on Y)
                anim_params[Y_OFFSET] += math.sin(t * 1.5) * 3
                # Subtle pupil pulse
                pulse = 1.0 + math.sin(t * 0.5) * 0.05
                anim_params[PUPIL_SCALE_LEFT] *= pulse
                anim_params[PUPIL_SCALE_RIGHT] *= pulse
                
            elif "LAUGHING" in self.current_emotion_name:
                # High frequency shake
                anim_params[Y_OFFSET] += math.sin(t * 20) * 5
                # Mouth pulse
                anim_params[MOUTH_OPEN] += math.sin(t * 15) * 10
                
            elif "ECSTATIC" in self.current_emotion_name:
                # Pupil "shining" (fast scale pulse)
                pulse = 1.0 + math.sin(t * 10) * 0.15
                anim_params[PUPIL_SCALE_LEFT] *= pulse
                anim_params[PUPIL_SCALE_RIGHT] *= pulse
                # Bouncing
                anim_params[Y_OFFSET] += abs(math.sin(t * 8)) * -10
                
            elif "THINKING" in self.current_emotion_name:
                # Eyes moving left-right
                anim_params[X_SHIFT] += math.sin(t * 2) * 20

            elif "WINK" in self.current_emotion_name:
                # Keep the open eye lively during the wink pose
                if self.current_emotion_name == "WINK_RIGHT":
                    anim_params[PUPIL_SCALE_LEFT] *= (1.0 + math.sin(t * 6) * 0.04)
                    anim_params[BROW_TILT_LEFT] -= math.sin(t * 4) * 2
                    anim_params[BROW_TILT_RIGHT] += math.sin(t * 4) * 2
                else:
                    anim_params[PUPIL_SCALE_RIGHT] *= (1.0 + math.sin(t * 6) * 0.04)
                    anim_params[BROW_TILT_LEFT] += math.sin(t * 4) * 2
                    anim_params[BROW_TILT_RIGHT] -= math.sin(t * 4) * 2

        # 3. Render and Publish
        img = render_face_image(anim_params, canvas_size=self.canvas_size)
        ros_msg = self.bridge.cv2_to_imgmsg(np.array(img), encoding="rgb8")
        self.image_pub.publish(ros_msg)

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
