from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'dev',
            default_value='/dev/input/js0',
            description='Joystick device file'
        ),
        DeclareLaunchArgument(
            'deadzone',
            default_value='0.05',
            description='Joystick deadzone'
        ),
        DeclareLaunchArgument(
            'autorepeat_rate',
            default_value='20.0',
            description='Joystick autorepeat rate'
        ),
        
        Node(
            package='joy_linux',
            executable='joy_node',
            name='joystick_driver',
            parameters=[{
                'dev': LaunchConfiguration('dev'),
                'deadzone': LaunchConfiguration('deadzone'),
                'autorepeat_rate': LaunchConfiguration('autorepeat_rate')
            }]
        )
    ])
