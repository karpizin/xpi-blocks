import json
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Example Config for a Soil Moisture Sensor (Address 1)
    # Reads Register 0 (Moisture) and 1 (Temperature)
    reg_map = [
        {"name": "moisture", "register": 0, "decimals": 1, "function_code": 3},
        {"name": "temperature", "register": 1, "decimals": 1, "function_code": 3}
    ]

    return LaunchDescription([
        Node(
            package='xpi_comms',
            executable='modbus_rtu_node.py',
            name='modbus_reader',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baudrate': 9600,
                'slave_address': 1,
                'polling_rate': 1.0,
                'register_map': json.dumps(reg_map)
            }]
        )
    ])
