from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import json

def generate_launch_description():
    # Use environment variable for sensor configurations, as it can be complex JSON
    # Example: export XPI_SENSOR_CONFIGS='[{"channel": 0, "type": "thermistor", "R_ref": 10000.0, "B_const": 3950.0, "V_supply": 3.3}, {"channel": 1, "type": "ldr", "R_ref": 10000.0, "V_supply": 3.3, "lux_factor": 500000.0}]'
    sensor_configs_env_var = os.environ.get('XPI_SENSOR_CONFIGS', '[]')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'adc_node_name',
            default_value='ads1115_adc',
            description='ROS2 name of the ADS1115 driver node (e.g., "ads1115_adc")'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='10.0',
            description='Rate in Hz to publish interpreted sensor readings'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='analog_sensors_link',
            description='Frame ID for sensor_msgs messages'
        ),
        DeclareLaunchArgument(
            'sensor_configs_json',
            default_value=sensor_configs_env_var, # Pass as JSON string
            description='JSON string containing configurations for each analog sensor'
        ),
        
        Node(
            package='xpi_sensors',
            executable='analog_sensor_interpreter_node',
            name='analog_sensor_interpreter',
            parameters=[{
                'adc_node_name': LaunchConfiguration('adc_node_name'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'frame_id': LaunchConfiguration('frame_id'),
                'sensor_configs_json': LaunchConfiguration('sensor_configs_json')
            }],
            output='screen'
        )
    ])
