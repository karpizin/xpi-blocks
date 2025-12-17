#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32, Int32

class JoyMapperNode(Node):
    def __init__(self):
        super().__init__('joy_mapper_node')

        # --- Parameters ---
        # 1. Drive Configuration
        self.declare_parameter('enable_drive', True)
        self.declare_parameter('drive_type', 'arcade') # arcade, tank (TODO)
        self.declare_parameter('axis_linear', 1)       # Left Stick Y
        self.declare_parameter('axis_angular', 3)      # Right Stick X (standard usually)
        self.declare_parameter('scale_linear', 1.0)
        self.declare_parameter('scale_angular', 1.0)
        self.declare_parameter('deadzone', 0.05)

        # 2. Button/Axis Bindings
        # Format: List of strings "type:index:topic:msg_type:scale/toggle_state"
        # Ideally this would be a complex dict param, but ROS2 params are simpler as lists of strings
        # Examples:
        # "toggle_btn:0:light_switch:Bool"
        # "momentary_btn:1:horn:Bool"
        # "axis:5:gripper:Float32:180.0" (axis val * 180)
        self.declare_parameter('bindings', [])

        # --- State ---
        self.drive_enabled = self.get_parameter('enable_drive').value
        self.bindings_config = self.get_parameter('bindings').value
        self.publishers_map = {}
        self.toggle_states = {} # Keep track of toggle button states (False/True)
        self.prev_buttons = []  # To detect rising edge

        # --- Setup Drive Publisher ---
        if self.drive_enabled:
            self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
            self.get_logger().info("Drive mixing enabled (cmd_vel)")

        # --- Parse Bindings ---
        self.parsed_bindings = []
        for binding in self.bindings_config:
            try:
                parts = binding.split(':')
                b_type = parts[0]
                index = int(parts[1])
                topic = parts[2]
                msg_type = parts[3]
                param = float(parts[4]) if len(parts) > 4 else 1.0

                # Create Publisher if not exists
                if topic not in self.publishers_map:
                    if msg_type == 'Bool':
                        pub = self.create_publisher(Bool, topic, 10)
                    elif msg_type == 'Float32':
                        pub = self.create_publisher(Float32, topic, 10)
                    elif msg_type == 'Int32':
                        pub = self.create_publisher(Int32, topic, 10)
                    else:
                        self.get_logger().warn(f"Unknown msg type {msg_type}")
                        continue
                    self.publishers_map[topic] = pub

                self.parsed_bindings.append({
                    'type': b_type,
                    'index': index,
                    'topic': topic,
                    'pub': self.publishers_map[topic],
                    'param': param
                })
                
                # Init toggle state
                if b_type == 'toggle_btn':
                    self.toggle_states[index] = False
                
                # Init axis as btn state
                if b_type == 'axis_as_btn':
                    self.axis_states[index] = False

            except Exception as e:
                self.get_logger().error(f"Failed to parse binding '{binding}': {e}")

    def joy_callback(self, msg):
        # Init prev_buttons on first run
        if not self.prev_buttons:
            self.prev_buttons = [0] * len(msg.buttons)
            # Init axis states defaults if not set (safe guard)
            if not hasattr(self, 'axis_states'): self.axis_states = {}

        # --- 1. Drive Handling ---
        if self.drive_enabled:
            twist = Twist()
            
            # Read Axes
            lin_val = msg.axes[self.get_parameter('axis_linear').value]
            ang_val = msg.axes[self.get_parameter('axis_angular').value]

            # Deadzone
            dz = self.get_parameter('deadzone').value
            if abs(lin_val) < dz: lin_val = 0.0
            if abs(ang_val) < dz: ang_val = 0.0

            # Scale
            twist.linear.x = lin_val * self.get_parameter('scale_linear').value
            twist.angular.z = ang_val * self.get_parameter('scale_angular').value
            
            self.pub_cmd_vel.publish(twist)

        # --- 2. Bindings Handling ---
        for item in self.parsed_bindings:
            b_type = item['type']
            idx = item['index']
            pub = item['pub']
            param = item['param']

            # --- Buttons ---
            if 'btn' in b_type:
                if idx >= len(msg.buttons): continue
                
                current_state = msg.buttons[idx]
                prev_state = self.prev_buttons[idx] if idx < len(self.prev_buttons) else 0
                rising_edge = (current_state == 1 and prev_state == 0)

                if b_type == 'momentary_btn':
                    m = Bool()
                    m.data = bool(current_state)
                    pub.publish(m)

                elif b_type == 'toggle_btn':
                    if rising_edge:
                        self.toggle_states[idx] = not self.toggle_states[idx]
                        m = Bool()
                        m.data = self.toggle_states[idx]
                        pub.publish(m)
                        self.get_logger().info(f"Toggled {item['topic']} to {m.data}")

            # --- Axes ---
            elif 'axis' in b_type:
                if idx >= len(msg.axes): continue
                val = msg.axes[idx]

                if b_type == 'axis':
                    # Scale output
                    out_val = val * param
                    m = Float32()
                    m.data = float(out_val)
                    pub.publish(m)
                
                elif b_type == 'axis_as_btn':
                    # Threshold check (param is threshold)
                    # Example: If stick > 0.5 -> True
                    new_state = (val > param)
                    
                    # Publish only on change to avoid spam, or continuous?
                    # Switches on RC stay in position.
                    # Let's publish on change.
                    if idx not in self.axis_states: self.axis_states[idx] = not new_state # Force update first time
                    
                    if new_state != self.axis_states.get(idx):
                        self.axis_states[idx] = new_state
                        m = Bool()
                        m.data = new_state
                        pub.publish(m)
                        self.get_logger().info(f"Axis Switch {item['topic']} -> {new_state}")

        # Update History
        self.prev_buttons = msg.buttons

def main(args=None):
    rclpy.init(args=args)
    node = JoyMapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
