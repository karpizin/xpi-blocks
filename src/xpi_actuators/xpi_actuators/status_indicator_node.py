#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, ColorRGBA, Float32
import time

class StatusIndicatorNode(Node):
    def __init__(self):
        super().__init__('status_indicator_node')
        
        # Status Dictionary: (Priority, RGB_Color, Pattern)
        # Priority: 0-100 (higher is more important)
        self.status_db = {
            'ERROR_CRITICAL': (100, (255, 0, 0), 'sos'),
            'LOW_BATTERY':    (80, (255, 100, 0), 'double_blink'),
            'GPS_SEARCH':     (60, (0, 255, 255), 'fast_blink'),
            'AI_THINKING':    (50, (255, 0, 255), 'breathe'),
            'CONNECTED':      (40, (0, 0, 255), 'status_pulse'),
            'GPS_FIXED':      (30, (0, 255, 255), 'solid'),
            'SYSTEM_OK':      (10, (0, 255, 0), 'solid'),
            'STANDBY':        (5, (0, 50, 0), 'breathe'),
        }

        # Currently active statuses { 'NAME': timestamp_started }
        self.active_statuses = {'SYSTEM_OK': time.time()}
        
        # Multiplexing variables
        self.cycle_index = 0
        self.last_oneshot_time = 0
        self.oneshot_active = False

        # Publishers
        self.effect_pub = self.create_publisher(String, '/ws2812/set_effect', 10)
        self.color_pub = self.create_publisher(ColorRGBA, '/ws2812/set_color', 10)
        self.speed_pub = self.create_publisher(Float32, '/ws2812/set_speed', 10)

        # Subscribers
        self.create_subscription(String, '/status/set', self.cb_set_status, 10)
        self.create_subscription(String, '/status/clear', self.cb_clear_status, 10)
        
        # Main logic timer (5 Hz — sufficient for layer switching)
        self.create_timer(0.2, self.update_logic)
        
        self.get_logger().info('Advanced Multiplexing Status Indicator Node started.')

    def cb_set_status(self, msg):
        status = msg.data.upper()
        if status in self.status_db:
            # If it's a new status, we can trigger a oneshot override
            if status not in self.active_statuses:
                self.trigger_oneshot()
            
            self.active_statuses[status] = time.time()
            self.get_logger().info(f'Status ACTIVE: {status}')
        else:
            self.get_logger().warn(f'Unknown status: {status}')

    def cb_clear_status(self, msg):
        status = msg.data.upper()
        if status in self.active_statuses:
            del self.active_statuses[status]
            self.get_logger().info(f'Status CLEARED: {status}')

    def trigger_oneshot(self):
        self.oneshot_active = True
        self.last_oneshot_time = time.time()

    def update_logic(self):
        if not self.active_statuses:
            return

        # 1. Get list of active statuses sorted by priority
        sorted_active = sorted(
            self.active_statuses.keys(), 
            key=lambda x: self.status_db[x][0], 
            reverse=True
        )

        target_status = sorted_active[0] # Default to the most important

        # 2. Interjections Logic
        # Every few ticks, show the next status in priority order
        self.cycle_index += 1
        if self.cycle_index % 10 == 0: # Every 2 seconds (at 5Hz timer)
            # Cycle through 2nd, 3rd, or 4th status
            secondary_count = min(len(sorted_active) - 1, 3)
            if secondary_count > 0:
                sub_idx = (self.cycle_index // 10) % secondary_count + 1
                target_status = sorted_active[sub_idx]
                # Force 'flash' effect for a brief interjection
                self.publish_status(target_status, override_effect='flash')
                return

        # 3. Oneshot Logic (when a new status is activated — show for 1 sec)
        if self.oneshot_active:
            if time.time() - self.last_oneshot_time < 1.0:
                self.publish_status(sorted_active[0]) # Show the newest/most important
                return
            else:
                self.oneshot_active = False

        # 4. Normal Mode (dominant status)
        self.publish_status(target_status)

    def publish_status(self, status_name, override_effect=None):
        priority, color, effect = self.status_db[status_name]
        if override_effect:
            effect = override_effect

        # Send to driver
        c_msg = ColorRGBA()
        c_msg.r, c_msg.g, c_msg.b, c_msg.a = color[0]/255.0, color[1]/255.0, color[2]/255.0, 1.0
        self.color_pub.publish(c_msg)
        
        e_msg = String()
        e_msg.data = effect
        self.effect_pub.publish(e_msg)
        
        # Use standard or adaptive speed
        s_msg = Float32()
        s_msg.data = 1.0
        self.speed_pub.publish(s_msg)

def main(args=None):
    rclpy.init(args=args)
    node = StatusIndicatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
