#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import cv2
import numpy as np
import mediapipe as mp
from collections import deque
import time

class GestureClassifier:
    def __init__(self):
        self.mp_hands = mp.solutions.hands
        self.mp_draw = mp.solutions.drawing_utils
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2, # Support 2 hands
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        
        # Landmark Indices
        self.WRIST = 0
        self.THUMB_CMC = 1
        self.THUMB_MCP = 2
        self.THUMB_IP = 3
        self.THUMB_TIP = 4
        self.INDEX_MCP = 5
        self.INDEX_TIP = 8
        self.MIDDLE_MCP = 9
        self.MIDDLE_TIP = 12
        self.RING_MCP = 13
        self.RING_TIP = 16
        self.PINKY_MCP = 17
        self.PINKY_TIP = 20

    def process(self, image):
        # Convert BGR to RGB
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = self.hands.process(image_rgb)
        return results

    def is_finger_up(self, lm, tip_idx, mcp_idx, is_thumb=False):
        wrist = lm[0]
        tip = lm[tip_idx]
        mcp = lm[mcp_idx]
        
        # Euclidean distance squared
        dist_tip = (tip.x - wrist.x)**2 + (tip.y - wrist.y)**2
        dist_mcp = (mcp.x - wrist.x)**2 + (mcp.y - wrist.y)**2
        
        return dist_tip > dist_mcp

    def classify(self, landmarks):
        lm = landmarks.landmark
        
        # Check Fingers
        thumb = self.is_finger_up(lm, 4, 2, is_thumb=True) # Tip vs MCP
        index = self.is_finger_up(lm, 8, 5)
        middle = self.is_finger_up(lm, 12, 9)
        ring = self.is_finger_up(lm, 16, 13)
        pinky = self.is_finger_up(lm, 20, 17)
        
        fingers = [thumb, index, middle, ring, pinky]
        count = sum(fingers)
        
        # Logic
        if count == 0: return "FIST"
        if count == 5: return "OPEN"
        if index and not middle and not ring and not pinky: return "POINTING"
        if thumb and not index and not middle: return "THUMB_UP" 
        
        return "UNKNOWN"

class GestureControlNode(Node):
    def __init__(self):
        super().__init__('gesture_control_node')

        # Parameters
        self.declare_parameter('mode', 'proportional') # discrete, proportional, joy
        self.declare_parameter('scale_linear', 0.5)
        self.declare_parameter('scale_angular', 1.0)
        self.declare_parameter('activation_gesture', 'FIST') # For proportional mode
        self.declare_parameter('image_topic', '/image_raw')

        self.mode = self.get_parameter('mode').value
        self.scale_lin = self.get_parameter('scale_linear').value
        self.scale_ang = self.get_parameter('scale_angular').value
        self.activation_gesture = self.get_parameter('activation_gesture').value
        img_topic = self.get_parameter('image_topic').value

        # Logic
        self.classifier = GestureClassifier()
        self.history_x = deque(maxlen=10) # For dynamic gestures (approx 0.5-1s history depending on FPS)
        self.last_swipe_time = 0.0
        self.swipe_cooldown = 1.0 # Seconds
        
        # Pubs
        self.pub_twist = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_joy = self.create_publisher(Joy, 'joy', 10)
        self.pub_gesture = self.create_publisher(String, '~/detected_gesture', 10)
        self.pub_debug_img = self.create_publisher(Image, '~/debug_image', 10)

        # Subs
        self.sub_img = self.create_subscription(Image, img_topic, self.img_callback, 10)
        
        self.get_logger().info(f"Gesture Control Started. Mode: {self.mode}")

    def detect_dynamic_gesture(self):
        if len(self.history_x) < 5: return None
        
        # Simple delta check
        # history holds normalized X (-1.0 to 1.0)
        start = self.history_x[0]
        end = self.history_x[-1]
        delta = end - start
        
        now = time.time()
        if now - self.last_swipe_time < self.swipe_cooldown:
            return None

        # Threshold for Swipe (e.g. moved 30% of screen width quickly)
        threshold = 0.6 # Since range is 2.0 (-1 to 1), 0.6 is 30% of screen
        
        if delta > threshold:
            self.last_swipe_time = now
            self.history_x.clear() # Reset
            return "SWIPE_RIGHT"
        elif delta < -threshold:
            self.last_swipe_time = now
            self.history_x.clear()
            return "SWIPE_LEFT"
            
        return None

    def img_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV
            h, w = msg.height, msg.width
            np_img = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 3))
            
            # Process
            results = self.classifier.process(np_img)
            
            gesture_name = "NONE"
            dynamic_gesture = None
            
            cmd = Twist()
            joy = Joy()
            joy.axes = [0.0]*4
            joy.buttons = [0]*5
            
            if results.multi_hand_landmarks:
                # Use PRIMARY hand (first detected) for control
                # TODO: Logic to pick "Right" hand for control if both present?
                # For now, index 0 is usually the most prominent/first detected.
                hand_landmarks = results.multi_hand_landmarks[0]
                
                # 1. Classify Static
                gesture_name = self.classifier.classify(hand_landmarks)
                
                # 2. Extract Center Position
                raw_x = hand_landmarks.landmark[9].x 
                raw_y = hand_landmarks.landmark[9].y
                
                norm_x = (raw_x - 0.5) * 2.0  # -1..1 
                norm_y = (0.5 - raw_y) * 2.0  # -1..1
                
                # 3. Dynamic Analysis
                self.history_x.append(norm_x)
                dynamic_gesture = self.detect_dynamic_gesture()
                if dynamic_gesture:
                    gesture_name = dynamic_gesture # Override for display/pub
                    self.get_logger().info(f"Dynamic Gesture: {dynamic_gesture}")

                # --- Mode Logic ---
                
                if self.mode == 'joy':
                    # Publish Raw
                    joy.axes[0] = norm_x
                    joy.axes[1] = norm_y
                    
                    # Buttons map to gestures
                    if gesture_name == 'FIST': joy.buttons[0] = 1
                    elif gesture_name == 'OPEN': joy.buttons[1] = 1
                    elif gesture_name == 'POINTING': joy.buttons[2] = 1
                    elif gesture_name == 'THUMB_UP': joy.buttons[3] = 1
                    
                    # Map swipes to buttons 
                    if dynamic_gesture == 'SWIPE_LEFT': joy.buttons[4] = 1 
                    elif dynamic_gesture == 'SWIPE_RIGHT': joy.buttons[5] = 1
                    
                    self.pub_joy.publish(joy)

                elif self.mode == 'proportional':
                    # Virtual Joystick
                    if gesture_name == self.activation_gesture:
                        cmd.linear.x = norm_y * self.scale_lin
                        cmd.angular.z = -norm_x * self.scale_ang # Invert X for turn
                    else:
                        cmd.linear.x = 0.0
                        cmd.angular.z = 0.0
                    self.pub_twist.publish(cmd)

                elif self.mode == 'discrete':
                    # Commands
                    if gesture_name == 'OPEN': 
                        pass 
                    elif gesture_name == 'POINTING': 
                        cmd.linear.x = self.scale_lin
                    elif gesture_name == 'FIST': 
                        cmd.linear.x = -self.scale_lin
                    elif gesture_name == 'THUMB_UP': 
                        cmd.angular.z = self.scale_ang
                    
                    self.pub_twist.publish(cmd)

            else:
                # No hand -> Stop and Clear History
                self.history_x.clear()
                if self.mode != 'joy':
                    self.pub_twist.publish(Twist())

            # Pub Status
            s = String()
            s.data = gesture_name
            self.pub_gesture.publish(s)

        except Exception as e:
            self.get_logger().error(f"Processing error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = GestureControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
