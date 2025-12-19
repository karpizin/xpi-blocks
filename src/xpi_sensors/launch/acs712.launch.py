import json
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. Config for ACS712-05B (5A Model) connected to A0
    sensor_config = [
        {
            "channel": 0,
            "type": "acs712",
            "sensitivity_mv_per_a": 185,  # 185mV/A for 5A model
            "offset_voltage": 2.5         # VCC/2 (Calibration point)
        }
    ]

    return LaunchDescription([
        # 1. Start the ADC Driver (Reads raw voltage)
        Node(
            package='xpi_sensors',
            executable='ads1115_node.py',
            name='ads1115_adc',
            output='screen',
            parameters=[{
                'i2c_bus': 1,
                'i2c_address': 0x48,
                'polling_rate': 10.0
            }]
        ),

        # 2. Start the Interpreter (Converts Voltage -> Amps)
        Node(
            package='xpi_sensors',
            executable='analog_sensor_interpreter_node.py',
            name='acs712_interpreter',
            output='screen',
            parameters=[{
                'adc_node_name': 'ads1115_adc',
                'publish_rate': 10.0,
                'sensor_configs_json': json.dumps(sensor_config)
            }]
        )
    ])
