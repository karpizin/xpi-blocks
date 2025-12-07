from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'i2c_bus',
            default_value='1',
            description='I2C Bus Number (usually 1 for Raspberry Pi)'
        ),
        DeclareLaunchArgument(
            'i2c_address',
            default_value='0x77', # BMP085/180 common address
            description='I2C Address of the BMP085/180 sensor'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='1.0',
            description='Rate in Hz to publish sensor readings'
        ),
        DeclareLaunchArgument(
            'oss',
            default_value='0', # Oversampling Setting (0-3)
            description='Oversampling Setting (0: Ultra Low Power, 1: Standard, 2: High, 3: Ultra High Resolution)'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='bmp085_link',
            description='Frame ID for sensor_msgs messages'
        ),
        DeclareLaunchArgument(
            'mock_hardware',
            default_value='false',
            description='Run in mock mode without real BMP085/180 hardware'
        ),
        
        Node(
            package='xpi_sensors',
            executable='bmp085_node',
            name='bmp085_env_sensor',
            parameters=[{
                'i2c_bus': LaunchConfiguration('i2c_bus'),
                'i2c_address': LaunchConfiguration('i2c_address'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'oss': LaunchConfiguration('oss'),
                'frame_id': LaunchConfiguration('frame_id'),
                'mock_hardware': LaunchConfiguration('mock_hardware')
            }],
            output='screen'
        )
    ])
