#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import base64
import threading
import time

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
        while rclpy.ok():
            try:
                self.get_logger().info(f'Connecting to NTRIP Caster {self.host}:{self.port}...')
                sock = socket.socket(socket.xml.parsers.expat.model.S_IFSOCK if hasattr(socket, 'S_IFSOCK') else socket.AF_INET, socket.SOCK_STREAM)
                sock.connect((self.host, self.port))

                # HTTP GET Request for NTRIP
                auth = base64.b64encode(f'{self.user}:{self.pw}'.encode()).decode()
                headers = (
                    f'GET /{self.mount} HTTP/1.0\r\n'
                    f'User-Agent: NTRIP XPI-Blocks Client\r\n'
                    f'Authorization: Basic {auth}\r\n'
                    f'Connection: close\r\n\r\n'
                )
                sock.sendall(headers.encode())

                while rclpy.ok():
                    data = sock.recv(2048)
                    if not data: break
                    
                    # Publish as hex string for topic safety
                    msg = String()
                    msg.data = data.hex()
                    self.corr_pub.publish(msg)
                
                sock.close()
            except Exception as e:
                self.get_logger().warn(f'NTRIP Connection error: {e}. Retrying in 5s...')
                time.sleep(5)

def main(args=None):
    rclpy.init(args=args)
    node = NtripClientNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
