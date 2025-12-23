#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature, RelativeHumidity

# Try to import Adafruit DHT library
try:
    import board
    import adafruit_dht
    HAS_DHT_LIB = True
except (ImportError, NotImplementedError):
    HAS_DHT_LIB = False

class DHTNode(Node):
    """
    ROS2 Node for DHT11 and DHT22 (AM2302) temperature and humidity sensors.
    Uses adafruit-circuitpython-dht library.
    Note: These sensors are slow. DHT11: 1Hz max, DHT22: 0.5Hz max.
    """
    def __init__(self):
        super().__init__('dht_node')
        
        # Parameters
        self.declare_parameter('sensor_type', 'DHT22') # 'DHT11' or 'DHT22'
        self.declare_parameter('gpio_pin', 4) # BCM pin number
        self.declare_parameter('polling_rate', 0.5) # Hz (Recommend 0.5 or lower for DHT22)
        self.declare_parameter('frame_id', 'dht_link')
        self.declare_parameter('mock_hardware', False)

        self.sensor_type = self.get_parameter('sensor_type').value.upper()
        self.pin_num = self.get_parameter('gpio_pin').value
        self.polling_rate = self.get_parameter('polling_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.mock_mode = self.get_parameter('mock_hardware').value

        # Publishers
        self.temp_pub = self.create_publisher(Temperature, '~/temperature', 10)
        self.hum_pub = self.create_publisher(RelativeHumidity, '~/humidity', 10)

        self.dht_device = None

        if not self.mock_mode and HAS_DHT_LIB:
            try:
                # Map integer pin to board object (e.g. 4 -> board.D4)
                pin_attr = f'D{self.pin_num}'
                if not hasattr(board, pin_attr):
                    self.get_logger().error(f"Pin GPIO{self.pin_num} not found in board library.")
                    return
                
                pin = getattr(board, pin_attr)

                if self.sensor_type == 'DHT11':
                    self.dht_device = adafruit_dht.DHT11(pin)
                elif self.sensor_type == 'DHT22' or self.sensor_type == 'AM2302':
                    self.dht_device = adafruit_dht.DHT22(pin)
                else:
                    self.get_logger().error(f"Unknown sensor type: {self.sensor_type}. Use DHT11 or DHT22.")
                    return
                
                self.get_logger().info(f"Initialized {self.sensor_type} on GPIO{self.pin_num}")

            except Exception as e:
                self.get_logger().error(f"Failed to init DHT sensor: {e}")
                self.dht_device = None
        else:
            if not self.mock_mode:
                self.get_logger().warn("adafruit_dht library not found. Falling back to MOCK mode.")
            else:
                self.get_logger().info("Running in MOCK mode.")
            self.mock_mode = True
            self.mock_temp = 20.0
            self.mock_hum = 50.0

        # Create Timer
        self.timer = self.create_timer(1.0 / self.polling_rate, self.timer_callback)

    def timer_callback(self):
        temp_c = 0.0
        humidity = 0.0
        read_success = False

        if self.mock_mode:
            # Mock Data Simulation
            import math
            t = time.monotonic()
            self.mock_temp = 20.0 + 2.0 * math.sin(t / 10.0)
            self.mock_hum = 50.0 + 10.0 * math.cos(t / 10.0)
            temp_c = self.mock_temp
            humidity = self.mock_hum
            read_success = True
        elif self.dht_device:
            try:
                # Read with retries is handled by lib, but we catch RuntimeError (checksum failure)
                temp_c = self.dht_device.temperature
                humidity = self.dht_device.humidity
                
                if temp_c is not None and humidity is not None:
                    read_success = True
                else:
                    self.get_logger().debug("DHT read returned None")

            except RuntimeError as error:
                # DHTs are notorious for CRC errors, this is normal. Just skip this frame.
                self.get_logger().debug(f"DHT Read Error (Normal): {error.args[0]}")
            except Exception as error:
                self.get_logger().error(f"DHT Critical Error: {error}")
                # Sometimes sensor hangs, re-init might be needed in severe cases, 
                # but usually we just wait for next cycle.

        if read_success:
            current_time = self.get_clock().now().to_msg()
            
            # Temperature Msg
            t_msg = Temperature()
            t_msg.header.stamp = current_time
            t_msg.header.frame_id = self.frame_id
            t_msg.temperature = float(temp_c)
            self.temp_pub.publish(t_msg)
            
            # Humidity Msg
            h_msg = RelativeHumidity()
            h_msg.header.stamp = current_time
            h_msg.header.frame_id = self.frame_id
            h_msg.relative_humidity = float(humidity) / 100.0 # Convert % to 0-1
            self.hum_pub.publish(h_msg)

            self.get_logger().debug(f"Published: T={temp_c:.1f}C H={humidity:.1f}%")

    def destroy_node(self):
        if self.dht_device:
            self.dht_device.exit()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DHTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
