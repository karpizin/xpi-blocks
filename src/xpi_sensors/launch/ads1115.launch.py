from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import json
import os

def generate_launch_description():
    # Convert string representation of list to actual list for channels
    channels_env_var = os.environ.get('XPI_ADS1115_CHANNELS', '[0]')
    channels_list = json.loads(channels_env_var)

    return LaunchDescription([
        DeclareLaunchArgument(
            'i2c_bus',
            default_value='1',
            description='I2C Bus Number (usually 1 for Raspberry Pi)'
        ),
        DeclareLaunchArgument(
            'i2c_address',
            default_value='0x48', # Default for ADS1115 (ADDR pin to GND)
            description='I2C Address of the ADS1115 sensor (0x48, 0x49, 0x4A, 0x4B)'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='10.0',
            description='Rate in Hz to publish sensor readings'
        ),
        DeclareLaunchArgument(
            'channels',
            default_value=channels_env_var, # Pass as string
            description='List of single-ended channels to read (0-3, e.g., "[0, 1, 3]")'
        ),
        DeclareLaunchArgument(
            'pga',
            default_value='2/3', # Programmable Gain Amplifier setting
            description='PGA setting (e.g., "2/3" for +/-6.144V, "1" for +/-4.096V, "2" for +/-2.048V, etc.)'
        ),
        DeclareLaunchArgument(
            'data_rate',
            default_value='128', # Samples per second
            description='Data rate in samples per second (8, 16, 32, 64, 128, 250, 475, 860)'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='ads1115_link',
            description='Frame ID for sensor_msgs messages'
        ),
        DeclareLaunchArgument(
            'mock_hardware',
            default_value='false',
            description='Run in mock mode without real ADS1115 hardware'
        ),
        
        Node(
            package='xpi_sensors',
            executable='ads1115_node',
            name='ads1115_adc',
            parameters=[{
                'i2c_bus': LaunchConfiguration('i2c_bus'),
                'i2c_address': LaunchConfiguration('i2c_address'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'channels': channels_list, # Pass as list of integers
                'pga': LaunchConfiguration('pga'),
                'data_rate': LaunchConfiguration('data_rate'),
                'frame_id': LaunchConfiguration('frame_id'),
                'mock_hardware': LaunchConfiguration('mock_hardware')
            }],
            output='screen'
        )
    ])
