#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import threading
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
import os
import json
import logging

# Disable Uvicorn logs to clear ROS2 console
logging.getLogger("uvicorn").setLevel(logging.WARNING)

class WebJoystickNode(Node):
    def __init__(self):
        super().__init__('web_joystick_node')

        # Parameters
        self.declare_parameter('port', 8080)
        self.declare_parameter('scale_linear', 1.0)
        self.declare_parameter('scale_angular', 1.5)
        
        self.port = self.get_parameter('port').value
        self.scale_lin = self.get_parameter('scale_linear').value
        self.scale_ang = self.get_parameter('scale_angular').value

        # Publishers
        self.pub_twist = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_joy = self.create_publisher(Joy, 'joy', 10) # Virtual Joy for Mapper

        # FastAPI App
        self.app = FastAPI()
        
        # Locate static file
        # In development: src/xpi_inputs/web_static/index.html
        # In install: share/xpi_inputs/web_static/index.html
        # We try both or rely on ament resource index, but simple path check is robust enough for python node.
        
        self.html_content = ""
        possible_paths = [
            os.path.join(os.path.dirname(__file__), '../../web_static/index.html'), # Dev
            os.path.join(os.getcwd(), 'src/xpi_inputs/web_static/index.html'), # From root
            '/opt/ros/humble/share/xpi_inputs/web_static/index.html', # Install (example)
            os.path.join(os.path.dirname(__file__), '../web_static/index.html') # Relative package
        ]
        
        # Dynamic lookup using ament_index is better but let's try direct read first
        # Actually, in ROS2 python packages, data_files put things in share/package/
        # But this script runs from lib/python/site-packages/...
        # Let's read from 'share' directory relative to executable?
        
        from ament_index_python.packages import get_package_share_directory
        try:
            share_dir = get_package_share_directory('xpi_inputs')
            html_path = os.path.join(share_dir, 'web_static', 'index.html')
            with open(html_path, 'r') as f:
                self.html_content = f.read()
            self.get_logger().info(f"Loaded Web UI from {html_path}")
        except Exception as e:
            self.get_logger().warn(f"Could not load HTML from share ({e}). Trying fallback.")
            self.html_content = "<h1>Error: HTML not found. Run 'colcon build' and source setup.bash</h1>"

        # Routes
        @self.app.get("/")
        async def get():
            return HTMLResponse(self.html_content)

        @self.app.websocket("/ws")
        async def websocket_endpoint(websocket: WebSocket):
            await websocket.accept()
            try:
                while True:
                    data = await websocket.receive_text()
                    cmd = json.loads(data)
                    self.process_command(cmd)
            except WebSocketDisconnect:
                pass
            except Exception as e:
                self.get_logger().warn(f"WS Error: {e}")

        # Start Server Thread
        self.server_thread = threading.Thread(target=self.run_server, daemon=True)
        self.server_thread.start()
        
        self.get_logger().info(f"Web Joystick listening on http://0.0.0.0:{self.port}")

    def run_server(self):
        uvicorn.run(self.app, host="0.0.0.0", port=self.port, log_level="warning")

    def process_command(self, cmd):
        # cmd = { lx: float, az: float }
        lx = float(cmd.get('lx', 0.0))
        az = float(cmd.get('az', 0.0))

        # 1. Publish Twist (Direct Drive)
        msg_twist = Twist()
        msg_twist.linear.x = lx * self.scale_lin
        msg_twist.angular.z = az * self.scale_ang
        self.pub_twist.publish(msg_twist)

        # 2. Publish Joy (For Mapper)
        # Map: Axis 1 (Left Y) -> lx, Axis 0 (Right X) -> -az (or similar)
        # Matches PS4 config roughly
        msg_joy = Joy()
        msg_joy.header.stamp = self.get_clock().now().to_msg()
        msg_joy.axes = [0.0] * 6
        msg_joy.buttons = [0] * 10
        
        msg_joy.axes[1] = lx  # Left Stick Y
        msg_joy.axes[3] = -az # Right Stick X (Inverted for consistency?)
        
        self.pub_joy.publish(msg_joy)

def main(args=None):
    rclpy.init(args=args)
    node = WebJoystickNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
