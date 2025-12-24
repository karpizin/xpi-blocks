from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('s0_pin', default_value='17'),
        DeclareLaunchArgument('s1_pin', default_value='27'),
        DeclareLaunchArgument('s2_pin', default_value='22'),
        DeclareLaunchArgument('s3_pin', default_value='10'),
        DeclareLaunchArgument('mock_hardware', default_value='false'),

        Node(
            package='xpi_sensors',
            executable='analog_mux_4067',
            name='analog_mux',
            namespace='analog_mux',
            parameters=[{
                's0_pin': LaunchConfiguration('s0_pin'),
                's1_pin': LaunchConfiguration('s1_pin'),
                's2_pin': LaunchConfiguration('s2_pin'),
                's3_pin': LaunchConfiguration('s3_pin'),
                'mock_hardware': LaunchConfiguration('mock_hardware'),
            }],
            output='screen'
        )
    ])
