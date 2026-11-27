#!/usr/bin/env python3
import os
import math
import random
import time

import numpy as np
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image as ImageMsg
from std_msgs.msg import String
from cv_bridge import CvBridge

from .expression_model import (
    apply_animation_overlay,
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
        self.blink_duration = 0.22 # seconds
        self.is_blinking = False
        self.blink_start = 0.0
        self.next_blink_at = time.time() + random.uniform(2.8, 5.0)
        
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
        blink_level = 0.0

        # A. Auto-Blink Logic (Global for most emotions)
        now = time.time()
        if not self.is_blinking and now >= self.next_blink_at and self.current_emotion_name not in {'WINK', 'WINK_RIGHT', 'BLINK'}:
            self.is_blinking = True
            self.blink_start = now

        if self.is_blinking:
            dt = now - self.blink_start
            progress = dt / self.blink_duration
            if progress < 1.0:
                blink_level = math.sin(progress * math.pi)
            else:
                self.is_blinking = False
                self.last_blink_time = now
                self.next_blink_at = now + random.uniform(2.8, 5.4)

        anim_params = apply_animation_overlay(
            self.current_params,
            self.current_emotion_name,
            t,
            blink_level=blink_level,
        )

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
