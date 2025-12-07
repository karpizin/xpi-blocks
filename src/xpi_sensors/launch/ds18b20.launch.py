from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'device_id',
            default_value='28-*',
            description='DS18B20 1-Wire device ID (e.g., 28-00000xxxxxxx, or 28-* for first found)'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='1.0',
            description='Rate in Hz to publish temperature readings'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='ds18b20_link',
            description='Frame ID for the sensor_msgs/Temperature message'
        ),
        DeclareLaunchArgument(
            'mock_hardware',
            default_value='false',
            description='Run in mock mode without real 1-Wire hardware'
        ),
        
        Node(
            package='xpi_sensors',
            executable='ds18b20_node',
            name='ds18b20_temp_sensor',
            parameters=[{
                'device_id': LaunchConfiguration('device_id'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'frame_id': LaunchConfiguration('frame_id'),
                'mock_hardware': LaunchConfiguration('mock_hardware')
            }],
            output='screen'
        )
    ])
