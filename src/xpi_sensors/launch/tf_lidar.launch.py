from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyS0'),
        DeclareLaunchArgument('model', default_value='tf02_pro'),
        DeclareLaunchArgument('mock_hardware', default_value='false'),

        Node(
            package='xpi_sensors',
            executable='tf_lidar_node',
            name='tf_lidar',
            namespace='tf_lidar',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'model': LaunchConfiguration('model'),
                'mock_hardware': LaunchConfiguration('mock_hardware'),
            }],
            output='screen'
        )
    ])
