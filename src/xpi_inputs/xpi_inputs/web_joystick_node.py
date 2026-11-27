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
import math
import time

# Disable Uvicorn logs to clear ROS2 console
logging.getLogger("uvicorn").setLevel(logging.WARNING)

class WebJoystickNode(Node):
    def __init__(self):
        super().__init__('web_joystick_node')

        # Parameters
        self.declare_parameter('port', 8080)
        self.declare_parameter('scale_linear', 1.0)
        self.declare_parameter('scale_angular', 1.5)
        # Dead-man: if no message arrives for cmd_timeout seconds while moving, publish a stop.
        # The page sends at 20 Hz, so 0.5 s means ~10 lost messages. 0 disables (not recommended).
        self.declare_parameter('cmd_timeout', 0.5)

        self.port = self.get_parameter('port').value
        self.scale_lin = self.get_parameter('scale_linear').value
        self.scale_ang = self.get_parameter('scale_angular').value
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)

        # Command state, shared between the web server thread and the ROS timer
        self._state_lock = threading.Lock()
        self._last_cmd_time = time.monotonic()
        self._moving = False

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
            self.get_logger().info("Web joystick client connected")
            try:
                while True:
                    data = await websocket.receive_text()
                    try:
                        cmd = json.loads(data)
                    except ValueError:
                        cmd = None
                    if not isinstance(cmd, dict):
                        # Malformed input never keeps the robot moving
                        self.get_logger().warn("Bad joystick message (expected a JSON object), stopping.")
                        self.stop()
                        continue
                    self.process_command(cmd)
            except WebSocketDisconnect:
                pass
            except Exception as e:
                self.get_logger().warn(f"WS Error: {e}")
            finally:
                # Connection lost or closed: stop right away instead of keeping the last command
                self.stop("client disconnected")

        if self.cmd_timeout > 0:
            self.create_timer(min(0.1, self.cmd_timeout / 5.0), self._watchdog_tick)
        else:
            self.get_logger().warn("cmd_timeout <= 0: web joystick command watchdog DISABLED.")

        # Start Server Thread
        self.server_thread = threading.Thread(target=self.run_server, daemon=True)
        self.server_thread.start()
        
        self.get_logger().info(f"Web Joystick listening on http://0.0.0.0:{self.port}")

    def run_server(self):
        uvicorn.run(self.app, host="0.0.0.0", port=self.port, log_level="warning")

    @staticmethod
    def _axis(value):
        """Joystick axis from the page: finite float clamped to [-1, 1]; anything else -> 0."""
        try:
            v = float(value)
        except (TypeError, ValueError):
            return 0.0
        if not math.isfinite(v):
            return 0.0
        return max(-1.0, min(1.0, v))

    def process_command(self, cmd):
        # cmd = { lx: float, az: float }, each in [-1, 1]
        lx = self._axis(cmd.get('lx', 0.0))
        az = self._axis(cmd.get('az', 0.0))
        with self._state_lock:
            self._last_cmd_time = time.monotonic()
            self._moving = lx != 0.0 or az != 0.0
        self._publish(lx, az)

    def stop(self, reason=None):
        """Publish a zero command. Logged only if the robot was being driven."""
        with self._state_lock:
            was_moving = self._moving
            self._moving = False
        if reason and was_moving:
            self.get_logger().warn(f"Web joystick: {reason}, stopping robot.")
        self._publish(0.0, 0.0)

    def _watchdog_tick(self):
        with self._state_lock:
            stale = self._moving and time.monotonic() - self._last_cmd_time > self.cmd_timeout
        if stale:
            self.stop(f"no command for {self.cmd_timeout:.2f} s")

    def destroy_node(self):
        self.stop()
        super().destroy_node()

    def _publish(self, lx, az):
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
