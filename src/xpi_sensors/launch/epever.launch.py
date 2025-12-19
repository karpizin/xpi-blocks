import json
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # EPEver Tracer Register Map (Input Registers - Function 04)
    # Scale 0.01 means raw 1200 -> 12.00V
    reg_map = [
        {"name": "pv_volt", "register": 0x3100, "decimals": 2, "function_code": 4, "scale": 0.01},
        {"name": "pv_amps", "register": 0x3101, "decimals": 2, "function_code": 4, "scale": 0.01},
        # "pv_power_L" at 0x3102. 
        {"name": "batt_volt", "register": 0x3104, "decimals": 2, "function_code": 4, "scale": 0.01},
        {"name": "batt_amps", "register": 0x3105, "decimals": 2, "function_code": 4, "scale": 0.01},
        {"name": "load_volt", "register": 0x310C, "decimals": 2, "function_code": 4, "scale": 0.01},
        {"name": "load_amps", "register": 0x310D, "decimals": 2, "function_code": 4, "scale": 0.01},
        {"name": "temp_ctrl", "register": 0x3111, "decimals": 2, "function_code": 4, "scale": 0.01},
        {"name": "soc",       "register": 0x311A, "decimals": 0, "function_code": 4, "scale": 1.0}
    ]

    return LaunchDescription([
        Node(
            package='xpi_comms',
            executable='modbus_rtu_node.py',
            name='epever',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baudrate': 115200,
                'slave_address': 1,
                'polling_rate': 0.5, # Slow poll for solar
                'register_map': json.dumps(reg_map)
            }]
        )
    ])
