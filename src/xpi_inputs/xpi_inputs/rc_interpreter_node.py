#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32, Int32
import math

class RCInterpreterNode(Node):
    """
    ROS2 Node that interprets raw Joy input from RC receivers (SBUS, CRSF, PPM).
    Maps channels to robot commands with support for expo, rates, and mixing.
    """

    def __init__(self):
        super().__init__('rc_interpreter_node')

        # --- Parameters ---
        # 1. Input Source
        self.declare_parameter('input_joy_topic', '/sbus_receiver_node/joy')
        
        # 2. Channel Mapping (Indices in msg.axes)
        self.declare_parameter('ch_throttle', 2)  # Typically CH3
        self.declare_parameter('ch_steering', 0)  # Typically CH1
        self.declare_parameter('ch_yaw', 3)       # Typically CH4
        self.declare_parameter('ch_pitch', 1)     # Typically CH2
        
        # 3. Mixing & Scaling
        self.declare_parameter('drive_type', 'diff') # arcade, diff, independent
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 2.0)
        
        # 4. Expo & Deadzone
        # Expo: 0.0 (Linear) to 1.0 (Very soft at center)
        self.declare_parameter('expo_throttle', 0.0)
        self.declare_parameter('expo_steering', 0.2)
        self.declare_parameter('deadzone', 0.05)
        
        # 5. Switch Mapping (msg.buttons or high-index axes)
        # Format: "ch_index:topic:msg_type:threshold"
        self.declare_parameter('switch_bindings', [])

        # --- Read Config ---
        self.input_topic = self.get_parameter('input_joy_topic').value
        self.drive_type = self.get_parameter('drive_type').value
        self.max_lin = self.get_parameter('max_linear_speed').value
        self.max_ang = self.get_parameter('max_angular_speed').value
        self.deadzone = self.get_parameter('deadzone').value
        
        # --- Publishers ---
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Setup Switch Publishers
        self.switches = []
        for binding in self.get_parameter('switch_bindings').value:
            try:
                parts = binding.split(':')
                ch = int(parts[0])
                topic = parts[1]
                m_type = parts[2]
                thresh = float(parts[3]) if len(parts) > 3 else 0.5
                
                if m_type == 'Bool':
                    pub = self.create_publisher(Bool, topic, 10)
                elif m_type == 'Int32':
                    pub = self.create_publisher(Int32, topic, 10)
                else:
                    continue
                    
                self.switches.append({'ch': ch, 'pub': pub, 'type': m_type, 'thresh': thresh, 'state': None})
            except Exception as e:
                self.get_logger().error(f"Failed to parse switch binding {binding}: {e}")

        # --- Subscriber ---
        self.subscription = self.create_subscription(Joy, self.input_topic, self.joy_callback, 10)
        
        self.get_logger().info(f"RC Interpreter started. Listening on {self.input_topic}")

    def apply_expo(self, value, expo):
        """
        Applies exponential curve to the input.
        y = x * (expo * x^2 + (1 - expo))
        """
        if expo == 0: return value
        return value * (expo * (value**2) + (1 - expo))

    def apply_deadzone(self, value, threshold):
        if abs(value) < threshold: return 0.0
        # Re-scale to [0, 1] range after deadzone
        return (value - math.copysign(threshold, value)) / (1.0 - threshold)

    def joy_callback(self, msg):
        # 1. Extract Main Channels
        throttle_idx = self.get_parameter('ch_throttle').value
        steering_idx = self.get_parameter('ch_steering').value
        
        if throttle_idx >= len(msg.axes) or steering_idx >= len(msg.axes):
            return

        raw_throttle = msg.axes[throttle_idx]
        raw_steering = msg.axes[steering_idx]

        # 2. Apply Processing
        # Throttle
        throttle = self.apply_deadzone(raw_throttle, self.deadzone)
        throttle = self.apply_expo(throttle, self.get_parameter('expo_throttle').value)
        
        # Steering
        steering = self.apply_deadzone(raw_steering, self.deadzone)
        steering = self.apply_expo(steering, self.get_parameter('expo_steering').value)

        # 3. Mixing & Publishing
        twist = Twist()
        twist.linear.x = throttle * self.max_lin
        twist.angular.z = steering * self.max_ang
        self.cmd_vel_pub.publish(twist)

        # 4. Handle Switches
        for sw in self.switches:
            idx = sw['ch']
            if idx >= len(msg.axes): continue
            
            val = msg.axes[idx]
            new_state = (val > sw['thresh'])
            
            if new_state != sw['state']:
                sw['state'] = new_state
                if sw['type'] == 'Bool':
                    self.get_logger().info(f"RC Switch {idx} -> {new_state}")
                    sw['pub'].publish(Bool(data=new_state))
                elif sw['type'] == 'Int32':
                    sw['pub'].publish(Int32(data=1 if new_state else 0))

def main(args=None):
    rclpy.init(args=args)
    node = RCInterpreterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
