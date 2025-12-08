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
            default_value='0x48', # LM75A default address
            description='I2C Address of the LM75A sensor (0x48-0x4F)'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='1.0',
            description='Rate in Hz to publish temperature readings'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='lm75a_link',
            description='Frame ID for the sensor_msgs/Temperature message'
        ),
        DeclareLaunchArgument(
            'mock_hardware',
            default_value='false',
            description='Run in mock mode without real LM75A hardware'
        ),
        
        Node(
            package='xpi_sensors',
            executable='lm75a_node',
            name='lm75a_temp_sensor',
            parameters=[{
                'i2c_bus': LaunchConfiguration('i2c_bus'),
                'i2c_address': LaunchConfiguration('i2c_address'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'frame_id': LaunchConfiguration('frame_id'),
                'mock_hardware': LaunchConfiguration('mock_hardware')
            }],
            output='screen'
        )
    ])
