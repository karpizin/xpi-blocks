from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('pin', default_value='12', description='GPIO pin connected to ESC'),
        DeclareLaunchArgument('min_pulse', default_value='0.001', description='Min pulse width (s)'),
        DeclareLaunchArgument('max_pulse', default_value='0.002', description='Max pulse width (s)'),
        DeclareLaunchArgument('stop_value', default_value='-1.0', description='Value to send on stop (-1.0 for uni, 0.0 for bi)'),
        DeclareLaunchArgument('timeout', default_value='0.5', description='Safety timeout (s)'),

        Node(
            package='xpi_actuators',
            executable='esc_driver',
            name='esc_driver',
            parameters=[{
                'pin': LaunchConfiguration('pin'),
                'min_pulse': LaunchConfiguration('min_pulse'),
                'max_pulse': LaunchConfiguration('max_pulse'),
                'stop_value': LaunchConfiguration('stop_value'),
                'timeout': LaunchConfiguration('timeout')
            }],
            output='screen'
        )
    ])
