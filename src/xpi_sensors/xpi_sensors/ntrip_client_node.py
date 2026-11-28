#!/usr/bin/env python3
import base64
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class NtripClientNode(Node):
    """
    Connects to an NTRIP Caster to receive RTCM corrections.
    Publishes raw correction data to /rtk/corrections.
    """
    def __init__(self):
        super().__init__('ntrip_client_node')

        # 1. Parameters
        self.declare_parameter('host', 'rtk2go.com')
        self.declare_parameter('port', 2101)
        self.declare_parameter('mountpoint', '')
        self.declare_parameter('user', '')
        self.declare_parameter('password', '')

        self.host = self.get_parameter('host').value
        self.port = self.get_parameter('port').value
        self.mount = self.get_parameter('mountpoint').value
        self.user = self.get_parameter('user').value
        self.pw = self.get_parameter('password').value
        self.reconnect_delay_s = 5.0
        self._stop_event = threading.Event()

        # 2. Publisher
        self.corr_pub = self.create_publisher(String, '/rtk/corrections', 10)

        if not self.mount:
            self.get_logger().error('NTRIP Mountpoint not set!')
            return

        # 3. Connection Thread
        self.thread = threading.Thread(target=self.network_loop, daemon=True)
        self.thread.start()

    def network_loop(self):
        """Maintains connection to the NTRIP caster."""
        while rclpy.ok() and not self._stop_event.is_set():
            sock = None
            try:
                self.get_logger().info(f'Connecting to NTRIP Caster {self.host}:{self.port}...')
                sock = socket.create_connection((self.host, self.port), timeout=10.0)
                sock.settimeout(10.0)

                # HTTP GET Request for NTRIP
                auth = base64.b64encode(f'{self.user}:{self.pw}'.encode()).decode()
                headers = (
                    f'GET /{self.mount} HTTP/1.0\r\n'
                    f'User-Agent: NTRIP XPI-Blocks Client\r\n'
                    f'Authorization: Basic {auth}\r\n'
                    f'Connection: close\r\n\r\n'
                )
                sock.sendall(headers.encode())

                response_buffer = b''
                while b'\r\n\r\n' not in response_buffer:
                    chunk = sock.recv(1024)
                    if not chunk:
                        raise ConnectionError('NTRIP caster closed connection before sending headers.')
                    response_buffer += chunk

                header_bytes, payload = response_buffer.split(b'\r\n\r\n', 1)
                header_text = header_bytes.decode(errors='replace')
                if '200 OK' not in header_text and not header_text.startswith('ICY 200'):
                    raise ConnectionError(f'Unexpected NTRIP response: {header_text.splitlines()[0]}')

                if payload:
                    msg = String()
                    msg.data = payload.hex()
                    self.corr_pub.publish(msg)

                while rclpy.ok() and not self._stop_event.is_set():
                    data = sock.recv(2048)
                    if not data:
                        break
                    
                    # Publish as hex string for topic safety
                    msg = String()
                    msg.data = data.hex()
                    self.corr_pub.publish(msg)
            except socket.timeout:
                self.get_logger().warning(f'NTRIP socket timed out. Retrying in {self.reconnect_delay_s:.0f}s...')
            except Exception as e:
                self.get_logger().warning(f'NTRIP connection error: {e}. Retrying in {self.reconnect_delay_s:.0f}s...')
            finally:
                if sock is not None:
                    try:
                        sock.close()
                    except OSError:
                        pass

            if not self._stop_event.is_set():
                time.sleep(self.reconnect_delay_s)

    def destroy_node(self):
        self._stop_event.set()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = NtripClientNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
