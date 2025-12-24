import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
import time
import threading

# Try to import the hardware library
try:
    from pyLoRa import LoRa, BOARD
    HAS_HARDWARE = True
except ImportError:
    HAS_HARDWARE = False

class LoRaRawNode(Node):
    """
    ROS2 Node for SX1276/SX1278 Raw LoRa communication via SPI.
    """

    def __init__(self):
        super().__init__('lora_raw_node')

        # Parameters
        self.declare_parameter('frequency', 433.0) # MHz
        self.declare_parameter('spreading_factor', 7)
        self.declare_parameter('bandwidth', 125000) # Hz
        self.declare_parameter('spi_bus', 0)
        self.declare_parameter('spi_cs', 1)
        self.declare_parameter('dio0_pin', 25)
        self.declare_parameter('reset_pin', 17)
        self.declare_parameter('mock_hardware', not HAS_HARDWARE)

        freq = self.get_parameter('frequency').value
        sf = self.get_parameter('spreading_factor').value
        bw = self.get_parameter('bandwidth').value
        mock_mode = self.get_parameter('mock_hardware').value

        # Hardware Setup
        if mock_mode:
            self.get_logger().warn('LoRa: Running in MOCK mode.')
        else:
            try:
                # Basic initialization using pyLoRa (logic may vary slightly by library version)
                # We assume standard Raspberry Pi pinout
                self.lora = LoRa(
                    bus=self.get_parameter('spi_bus').value,
                    client=self.get_parameter('spi_cs').value,
                    freq=freq,
                    sf=sf,
                    bw=bw
                )
                self.lora.set_mode(LoRa.MODE_SLEEP)
                self.lora.set_pa_config(pa_select=1)
                self.lora.set_mode(LoRa.MODE_RXCONT)
                self.get_logger().info(f'LoRa initialized at {freq}MHz, SF{sf}')
            except Exception as e:
                self.get_logger().error(f'Failed to initialize LoRa hardware: {e}')
                mock_mode = True

        self.mock_mode = mock_mode

        # Publishers
        self.rx_pub = self.create_publisher(String, '~/rx', 10)
        self.rssi_pub = self.create_publisher(Int32, '~/rssi', 10)

        # Subscribers
        self.tx_sub = self.create_subscription(String, '~/tx', self.tx_callback, 10)

        # Start RX Background Thread
        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()

    def tx_callback(self, msg):
        if self.mock_mode:
            self.get_logger().info(f"MOCK: Transmitting LoRa packet: {msg.data}")
            return

        try:
            self.get_logger().info(f"Transmitting: {msg.data}")
            self.lora.set_mode(LoRa.MODE_TX)
            self.lora.write_payload(list(msg.data.encode('utf-8')))
            time.sleep(0.1) # Wait for transmission
            self.lora.set_mode(LoRa.MODE_RXCONT) # Return to listening
        except Exception as e:
            self.get_logger().error(f"LoRa TX error: {e}")

    def rx_loop(self):
        while rclpy.ok():
            if self.mock_mode:
                time.sleep(5)
                continue

            try:
                if self.lora.received_packet():
                    payload = self.lora.read_payload()
                    rssi = self.lora.get_pkt_rssi_value()
                    
                    data_str = bytes(payload).decode('utf-8', errors='ignore')
                    
                    # Publish data
                    msg = String()
                    msg.data = data_str
                    self.rx_pub.publish(msg)
                    
                    # Publish RSSI
                    rssi_msg = Int32()
                    rssi_msg.data = int(rssi)
                    self.rssi_pub.publish(rssi_msg)
                    
                    self.get_logger().info(f"Received Packet: {data_str} (RSSI: {rssi})")
            except Exception as e:
                # self.get_logger().error(f"LoRa RX error: {e}")
                pass
            time.sleep(0.01)

    def destroy_node(self):
        if not self.mock_mode:
            self.lora.set_mode(LoRa.MODE_SLEEP)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LoRaRawNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
