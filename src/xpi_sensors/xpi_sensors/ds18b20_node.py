import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature
import os
import glob
import time

class DS18B20Node(Node):
    """
    ROS2 Node for reading temperature from DS18B20 1-Wire sensors.
    Publishes sensor_msgs/Temperature.
    """

    def __init__(self):
        super().__init__('ds18b20_node')

        # 1. Declare Parameters
        self.declare_parameter('device_id', '28-*') # Wildcard for first found device, or specific ID
        self.declare_parameter('publish_rate', 1.0) # Hz
        self.declare_parameter('frame_id', 'ds18b20_link')
        self.declare_parameter('mock_hardware', False) # For testing without real 1-Wire

        # 2. Read Parameters
        self.device_id = self.get_parameter('device_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.mock_mode = self.get_parameter('mock_hardware').value

        self.device_file = None
        if not self.mock_mode:
            self.base_dir = '/sys/bus/w1/devices/'
            # Search for the device file
            device_folder = glob.glob(self.base_dir + self.device_id)
            if device_folder:
                self.device_file = device_folder[0] + '/w1_slave'
                self.get_logger().info(f'DS18B20: Found device file {self.device_file}')
            else:
                self.get_logger().error(f'DS18B20: Device with ID "{self.device_id}" not found. '
                                        'Make sure 1-Wire is enabled and sensor is connected. Falling back to mock mode.')
                self.mock_mode = True

        if self.mock_mode:
            self.get_logger().warn('DS18B20: Running in MOCK mode. No real 1-Wire data will be read.')
            self.mock_temp = 25.0
            self.mock_temp_direction = 1

        # 3. Publisher
        self.temp_publisher = self.create_publisher(Temperature, '~/temperature', 10)
        
        # 4. Timer for publishing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        self.get_logger().info(f'DS18B20: Publishing temperature at {self.publish_rate} Hz.')

    def read_temp_raw(self):
        """Reads raw data from the DS18B20 device file."""
        if self.mock_mode:
            # Simulate temperature change
            self.mock_temp += self.mock_temp_direction * 0.1
            if self.mock_temp > 30.0 or self.mock_temp < 20.0:
                self.mock_temp_direction *= -1
            return ["YES", f"t={int(self.mock_temp * 1000)}"] # Format like real sensor output
        
        if not self.device_file:
            return None # Should not happen if mock_mode is handled

        try:
            with open(self.device_file, 'r') as f:
                lines = f.readlines()
            return lines
        except Exception as e:
            self.get_logger().error(f'DS18B20: Failed to read from device file: {e}')
            return None

    def read_temp(self):
        """Parses raw data and returns temperature in Celsius."""
        lines = self.read_temp_raw()
        if not lines:
            return None

        # Example output:
        # 1c 01 4b 46 7f ff 0e 10 32 : crc=32 YES
        # 1c 01 4b 46 7f ff 0e 10 32 t=28750
        
        while lines[0].strip()[-3:] != 'YES':
            time.sleep(0.2)
            lines = self.read_temp_raw()
            if not lines: return None # handle if read fails again

        equals_pos = lines[1].find('t=')
        if equals_pos != -1:
            temp_string = lines[1][equals_pos+2:]
            temp_c = float(temp_string) / 1000.0
            return temp_c
        return None

    def timer_callback(self):
        """Publishes temperature message."""
        temperature_c = self.read_temp()
        if temperature_c is not None:
            temp_msg = Temperature()
            temp_msg.header.stamp = self.get_clock().now().to_msg()
            temp_msg.header.frame_id = self.frame_id
            temp_msg.temperature = float(temperature_c)
            temp_msg.variance = 0.0 # DS18B20 typically high precision, low variance
            
            self.temp_publisher.publish(temp_msg)
            self.get_logger().debug(f'Published temperature: {temperature_c:.2f} °C')
        else:
            self.get_logger().warn('DS18B20: Could not read temperature.')

def main(args=None):
    rclpy.init(args=args)
    node = DS18B20Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
