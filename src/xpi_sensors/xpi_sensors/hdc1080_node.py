#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature, RelativeHumidity
import time
try:
    from smbus2 import SMBus
except ImportError:
    SMBus = None

class HDC1080Node(Node):
    def __init__(self):
        super().__init__('hdc1080_node')
        
        # Parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x40)
        self.declare_parameter('update_rate', 1.0)
        
        self.bus_num = self.get_parameter('i2c_bus').value
        self.addr = self.get_parameter('i2c_address').value
        update_rate = self.get_parameter('update_rate').value
        
        # Hardware setup
        self.bus = None
        if SMBus:
            try:
                self.bus = SMBus(self.bus_num)
                # Initialize HDC1080: Config register 0x02
                # Bit 10: 1 (Sequential mode), 8: 1 (Heater off)
                # Standard: 14-bit resolution for both
                self.bus.write_byte_data(self.addr, 0x02, 0x10)
                time.sleep(0.02)
                self.get_logger().info(f'HDC1080: Initialized on bus {self.bus_num} at 0x{self.addr:02x}')
            except Exception as e:
                self.get_logger().error(f'I2C Error: {e}')
        else:
            self.get_logger().warn('smbus2 not found. Running in MOCK mode.')

        # Publishers
        self.temp_pub = self.create_publisher(Temperature, '~/temperature', 10)
        self.hum_pub = self.create_publisher(RelativeHumidity, '~/humidity', 10)
        
        # Timer
        self.create_timer(1.0 / update_rate, self.read_data)

    def read_data(self):
        if not self.bus:
            # Mock data
            self.publish_data(22.5, 45.0)
            return

        try:
            # Trigger measurement by writing to 0x00
            self.bus.write_byte(self.addr, 0x00)
            time.sleep(0.02) # Wait for conversion
            
            # Read 4 bytes: [Temp High, Temp Low, Hum High, Hum Low]
            data = self.bus.read_i2c_block_data(self.addr, 0x00, 4)
            
            # Calculate Temperature: (Raw / 2^16) * 165 - 40
            temp_raw = (data[0] << 8) | data[1]
            temp = (temp_raw / 65536.0) * 165.0 - 40.0
            
            # Calculate Humidity: (Raw / 2^16) * 100
            hum_raw = (data[2] << 8) | data[3]
            hum = (hum_raw / 65536.0) * 100.0
            
            self.publish_data(temp, hum)
            
        except Exception as e:
            self.get_logger().error(f'Read error: {e}')

    def publish_data(self, temp, hum):
        now = self.get_clock().now().to_msg()
        
        t_msg = Temperature()
        t_msg.header.stamp = now
        t_msg.header.frame_id = 'hdc1080_link'
        t_msg.temperature = float(temp)
        self.temp_pub.publish(t_msg)
        
        h_msg = RelativeHumidity()
        h_msg.header.stamp = now
        h_msg.header.frame_id = 'hdc1080_link'
        h_msg.relative_humidity = float(hum) / 100.0 # Standard: 0.0 to 1.0
        self.hum_pub.publish(h_msg)

def main(args=None):
    rclpy.init(args=args)
    node = HDC1080Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
