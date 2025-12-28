#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, ColorRGBA, Float32

class StatusIndicatorNode(Node):
    def __init__(self):
        super().__init__('status_indicator_node')
        
        # Mapping USIS logic to LED effects
        # Key: Status Name, Value: (Effect, ColorRGB, Speed)
        self.status_map = {
            'SYSTEM_OK':    ('solid', (0, 255, 0), 1.0),
            'BOOT':         ('blink', (255, 255, 255), 2.0),
            'STANDBY':      ('breathe', (0, 255, 0), 0.5),
            'LOW_BATTERY':  ('double_blink', (255, 100, 0), 1.0),
            'ERROR_CRITICAL': ('sos', (255, 0, 0), 1.0),
            'AI_THINKING':  ('breathe', (255, 0, 255), 1.0),
            'GPS_SEARCH':   ('fast_blink', (0, 255, 255), 2.0),
            'GPS_FIXED':    ('solid', (0, 255, 255), 1.0),
            'CONNECTED':    ('status_pulse', (0, 0, 255), 1.0),
            'CALIBRATION':  ('alternating', (255, 255, 0), 2.0),
        }

        # Publishers to WS2812 Driver
        self.effect_pub = self.create_publisher(String, '/ws2812/set_effect', 10)
        self.color_pub = self.create_publisher(ColorRGBA, '/ws2812/set_color', 10)
        self.speed_pub = self.create_publisher(Float32, '/ws2812/set_speed', 10)

        # Subscriber for incoming status requests
        self.create_subscription(String, '/status/set', self.cb_set_status, 10)
        
        self.get_logger().info('Status Indicator Node (USIS) started.')

    def cb_set_status(self, msg):
        status = msg.data.upper()
        if status in self.status_map:
            effect, color, speed = self.status_map[status]
            
            # 1. Set Color
            c_msg = ColorRGBA()
            c_msg.r, c_msg.g, c_msg.b, c_msg.a = color[0]/255.0, color[1]/255.0, color[2]/255.0, 1.0
            self.color_pub.publish(c_msg)
            
            # 2. Set Speed
            s_msg = Float32()
            s_msg.data = float(speed)
            self.speed_pub.publish(s_msg)
            
            # 3. Set Effect (Trigger)
            e_msg = String()
            e_msg.data = effect
            self.effect_pub.publish(e_msg)
            
            self.get_logger().info(f'Status changed to: {status}')
        else:
            self.get_logger().warn(f'Unknown status requested: {status}')

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
