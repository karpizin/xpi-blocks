#!/usr/bin/env python3
import time
import json
import minimalmodbus
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ModbusRTUNode(Node):
    def __init__(self):
        super().__init__('modbus_rtu_node')
        
        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('slave_address', 1)
        self.declare_parameter('polling_rate', 1.0)
        self.declare_parameter('register_map', '[]') # JSON String

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.slave_addr = self.get_parameter('slave_address').value
        self.polling_rate = self.get_parameter('polling_rate').value
        self.reg_map_json = self.get_parameter('register_map').value
        
        try:
            self.reg_map = json.loads(self.reg_map_json)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Invalid JSON for register_map: {e}")
            self.reg_map = []

        # Publisher
        self.data_pub = self.create_publisher(String, '~/data', 10)

        # Init Modbus
        try:
            self.instrument = minimalmodbus.Instrument(self.port, self.slave_addr)
            self.instrument.serial.baudrate = self.baudrate
            self.instrument.serial.bytesize = 8
            self.instrument.serial.parity = serial.PARITY_NONE
            self.instrument.serial.stopbits = 1
            self.instrument.serial.timeout = 0.5
            self.instrument.mode = minimalmodbus.MODE_RTU
            self.instrument.clear_buffers_before_each_transaction = True
            
            self.get_logger().info(f"Modbus RTU initialized: {self.port} @ {self.baudrate}, Slave {self.slave_addr}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to open Serial Port: {e}")
            self.instrument = None

        # Timer
        self.timer = self.create_timer(1.0 / self.polling_rate, self.timer_callback)

    def timer_callback(self):
        if not self.instrument:
            return

        result = {}
        
        for reg in self.reg_map:
            name = reg.get('name', 'unknown')
            addr = reg.get('register')
            decimals = reg.get('decimals', 0)
            function = reg.get('function_code', 3)
            scale = reg.get('scale', 1.0)
            signed = reg.get('signed', False)
            
            try:
                val = self.instrument.read_register(addr, decimals, functioncode=function, signed=signed)
                # minimalmodbus handles decimals (division), but let's apply manual scale if needed
                # If decimals was 0, val is int.
                
                final_val = val * scale
                result[name] = final_val
                
            except Exception as e:
                self.get_logger().warn(f"Modbus Read Error ({name}): {e}")
                result[name] = None

        # Publish
        msg = String()
        msg.data = json.dumps(result)
        self.data_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ModbusRTUNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
