from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('i2c_bus', default_value='1'),
        DeclareLaunchArgument('publish_rate', default_value='10.0'),
        DeclareLaunchArgument('mode', default_value='2', description='1=Short, 2=Long'),
        DeclareLaunchArgument('mock_hardware', default_value='false'),

        Node(
            package='xpi_sensors',
            executable='vl53l1x_node',
            name='vl53l1x',
            namespace='vl53l1x',
            parameters=[{
                'i2c_bus': LaunchConfiguration('i2c_bus'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'mode': LaunchConfiguration('mode'),
                'mock_hardware': LaunchConfiguration('mock_hardware'),
            }],
            output='screen'
        )
    ])
