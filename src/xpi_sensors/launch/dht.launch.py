from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'sensor_type',
            default_value='DHT22',
            description='Sensor Type: DHT11 or DHT22'
        ),
        DeclareLaunchArgument(
            'gpio_pin',
            default_value='4',
            description='BCM GPIO Pin number (default GPIO 4)'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='0.5',
            description='Polling rate in Hz. Keep low! DHT22 max 0.5Hz (2s period), DHT11 max 1Hz.'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='dht_link',
            description='Frame ID for TF'
        ),
        DeclareLaunchArgument(
            'mock_hardware',
            default_value='false',
            description='Run in mock mode'
        ),
        
        Node(
            package='xpi_sensors',
            executable='dht_node',
            name='dht_sensor',
            parameters=[{
                'sensor_type': LaunchConfiguration('sensor_type'),
                'gpio_pin': LaunchConfiguration('gpio_pin'),
                'polling_rate': LaunchConfiguration('publish_rate'),
                'frame_id': LaunchConfiguration('frame_id'),
                'mock_hardware': LaunchConfiguration('mock_hardware')
            }],
            output='screen'
        )
    ])
