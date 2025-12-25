#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
import spidev
import time

class W25QxxNode(Node):
    """
    ROS2 Driver for W25Qxx series SPI Flash Memory.
    Supports W25Q16, W25Q32, W25Q64, W25Q128.
    """

    # W25Qxx Instructions
    CMD_WRITE_ENABLE = 0x06
    CMD_WRITE_DISABLE = 0x04
    CMD_READ_ID = 0x9F
    CMD_READ_DATA = 0x03
    CMD_PAGE_PROGRAM = 0x02
    CMD_SECTOR_ERASE = 0x20
    CMD_READ_STATUS_REG1 = 0x05

    def __init__(self):
        super().__init__('w25qxx_node')

        # Parameters
        self.declare_parameter('spi_bus', 0)
        self.declare_parameter('spi_device', 0)
        self.declare_parameter('spi_speed', 10000000) # 10MHz
        self.declare_parameter('mock_hardware', False)

        bus = self.get_parameter('spi_bus').value
        device = self.get_parameter('spi_device').value
        speed = self.get_parameter('spi_speed').value
        self.mock_mode = self.get_parameter('mock_hardware').value

        # Publishers
        self.data_pub = self.create_publisher(UInt8MultiArray, '~/data_out', 10)

        # Subscribers
        self.create_subscription(UInt8MultiArray, '~/write_raw', self._write_callback, 10)

        # SPI Init
        self.spi = spidev.SpiDev()
        if not self.mock_mode:
            try:
                self.spi.open(bus, device)
                self.spi.max_speed_hz = speed
                self.get_logger().info(f"SPI initialized on bus {bus}, device {device}")
                self.detect_chip()
            except Exception as e:
                self.get_logger().error(f"Failed to open SPI: {e}")
                self.mock_mode = True
        
        if self.mock_mode:
            self.get_logger().warn("Running in MOCK mode.")

    def detect_chip(self):
        # Read Manufacturer and Device ID
        # Cmd 0x9F: returns [MF7-MF0], [ID15-ID8], [ID7-ID0]
        resp = self.spi.xfer2([self.CMD_READ_ID, 0x00, 0x00, 0x00])
        mf_id = resp[1]
        mem_type = resp[2]
        capacity = resp[3]

        capacity_map = {
            0x15: "W25Q16 (2MB)",
            0x16: "W25Q32 (4MB)",
            0x17: "W25Q64 (8MB)",
            0x18: "W25Q128 (16MB)"
        }

        chip_name = capacity_map.get(capacity, f"Unknown (Cap ID: 0x{capacity:02X})")
        self.get_logger().info(f"Detected Flash: {chip_name}, Manufacturer ID: 0x{mf_id:02X}")

    def _wait_not_busy(self):
        if self.mock_mode: return
        while True:
            resp = self.spi.xfer2([self.CMD_READ_STATUS_REG1, 0x00])
            status = resp[1]
            if not (status & 0x01): # Check BUSY bit
                break
            time.sleep(0.001)

    def _write_enable(self):
        if self.mock_mode: return
        self.spi.xfer2([self.CMD_WRITE_ENABLE])

    def read_data(self, address, length):
        if self.mock_mode:
            return [0] * length
        
        # Read Data Cmd + 24-bit address
        cmd = [
            self.CMD_READ_DATA,
            (address >> 16) & 0xFF,
            (address >> 8) & 0xFF,
            address & 0xFF
        ]
        resp = self.spi.xfer2(cmd + [0x00] * length)
        return resp[4:]

    def _write_callback(self, msg):
        """
        Expects: [AddrHigh, AddrMid, AddrLow, Data0, Data1, ...]
        Warning: Flash MUST be erased before writing to a page.
        """
        if len(msg.data) < 4:
            self.get_logger().warn("Invalid write msg format. Need [A2, A1, A0, Data...]")
            return

        addr = (msg.data[0] << 16) | (msg.data[1] << 8) | msg.data[2]
        payload = list(msg.data[3:])

        self.get_logger().info(f"Flash Write: {len(payload)} bytes at 0x{addr:06X}")
        
        if self.mock_mode:
            return

        self._write_enable()
        
        # Page Program (supports up to 256 bytes)
        cmd = [
            self.CMD_PAGE_PROGRAM,
            (addr >> 16) & 0xFF,
            (addr >> 8) & 0xFF,
            addr & 0xFF
        ]
        self.spi.xfer2(cmd + payload)
        self._wait_not_busy()
        self.get_logger().info("Flash Write Complete.")

    def destroy_node(self):
        self.spi.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = W25QxxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
