#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import threading
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, HTMLResponse
import hmac
import ipaddress
import os
import json
import logging
import math
import secrets
import time
from urllib.parse import urlsplit

TOKEN_ENV = 'XPI_WEB_JOYSTICK_TOKEN'
MAX_MESSAGE_BYTES = 1024   # a command is ~30 bytes of JSON

# Disable Uvicorn logs to clear ROS2 console
logging.getLogger("uvicorn").setLevel(logging.WARNING)

class WebJoystickNode(Node):
    def __init__(self):
        super().__init__('web_joystick_node')

        # Parameters
        # Loopback by default: reachable only from the robot itself. Use host:=0.0.0.0 to
        # drive from a phone on the same network (a token is always required).
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 8080)
        # Access token; the XPI_WEB_JOYSTICK_TOKEN environment variable takes precedence.
        # Empty: a random token is generated at startup and the URL is printed to the log.
        self.declare_parameter('token', '')
        # Extra browser origins allowed to connect, comma separated (e.g. a dashboard at
        # 'http://192.168.1.10:3000'). The page served by this node is always allowed.
        self.declare_parameter('allowed_origins', '')
        self.declare_parameter('scale_linear', 1.0)
        self.declare_parameter('scale_angular', 1.5)
        # Dead-man: if no message arrives for cmd_timeout seconds while moving, publish a stop.
        # The page sends at 20 Hz, so 0.5 s means ~10 lost messages. 0 disables (not recommended).
        self.declare_parameter('cmd_timeout', 0.5)

        self.host = str(self.get_parameter('host').value)
        self.port = self.get_parameter('port').value
        self.allowed_origins = {o.strip().rstrip('/').lower()
                                for o in str(self.get_parameter('allowed_origins').value).split(',')
                                if o.strip()}
        token = os.environ.get(TOKEN_ENV) or str(self.get_parameter('token').value)
        self._token_generated = not token
        self.token = token or secrets.token_urlsafe(16)
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
        static_dir = None
        
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
            static_dir = os.path.dirname(html_path)
            self.get_logger().info(f"Loaded Web UI from {html_path}")
        except Exception as e:
            self.get_logger().warn(f"Could not load HTML from share ({e}). Trying fallback.")
            self.html_content = "<h1>Error: HTML not found. Run 'colcon build' and source setup.bash</h1>"

        # Routes
        @self.app.get("/")
        async def get():
            return HTMLResponse(self.html_content, headers={
                'Cache-Control': 'no-store', 'Referrer-Policy': 'no-referrer'})

        @self.app.get("/nipplejs.min.js")
        async def nipplejs():
            # Served locally: works without internet and no third-party script sees the token
            if static_dir is None:
                return HTMLResponse('not found', status_code=404)
            return FileResponse(os.path.join(static_dir, 'nipplejs.min.js'),
                                media_type='application/javascript')

        @self.app.websocket("/ws")
        async def websocket_endpoint(websocket: WebSocket):
            reason = self._reject_reason(websocket)
            if reason:
                client = websocket.client.host if websocket.client else '?'
                self.get_logger().warn(f"Web joystick: rejected connection from {client}: {reason}")
                await websocket.close(code=1008)   # policy violation; HTTP 403 before accept
                return
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
        
        self._log_access_url()

    def _log_access_url(self):
        url_host = f'[{self.host}]' if ':' in self.host else self.host
        if self.host in ('0.0.0.0', '::'):
            url_host = '<ROBOT_IP>'
        url = f'http://{url_host}:{self.port}/#token='
        if self._token_generated:
            self.get_logger().info(f"Web Joystick: open {url}{self.token}")
        else:
            self.get_logger().info(f"Web Joystick: open {url}<your token>")
        if not self._is_loopback(self.host):
            self.get_logger().warn(
                f"Web Joystick listens on {self.host}: anyone on this network who has the token "
                "can drive the robot, and plain HTTP is not encrypted. Use only on a trusted network.")

    @staticmethod
    def _is_loopback(host):
        if host == 'localhost':
            return True
        try:
            return ipaddress.ip_address(host).is_loopback
        except ValueError:
            return False

    def _reject_reason(self, websocket):
        """Why this WebSocket handshake must be refused, or None to accept it."""
        # 1. Origin: browsers always send it. A page from another site (or a DNS-rebinding
        #    attack) must not be able to open a socket to the robot (CSWSH). Clients that
        #    are not browsers send no Origin; they still need the token.
        origin = websocket.headers.get('origin')
        if origin is not None and not self._origin_allowed(origin, websocket.headers.get('host', '')):
            return f'origin {origin!r} not allowed'
        # 2. Token
        given = websocket.query_params.get('token', '')
        if not hmac.compare_digest(given.encode(), self.token.encode()):
            return 'missing or wrong token'
        return None

    def _origin_allowed(self, origin, host_header):
        origin = origin.strip().rstrip('/').lower()
        if origin in self.allowed_origins:
            return True
        parts = urlsplit(origin)
        # Same origin: the page this node served, opened as http://<host>:<port>/
        return parts.scheme in ('http', 'https') and parts.netloc == host_header.strip().lower()

    def run_server(self):
        uvicorn.run(self.app, host=self.host, port=self.port, log_level="warning",
                    ws_max_size=MAX_MESSAGE_BYTES)

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
