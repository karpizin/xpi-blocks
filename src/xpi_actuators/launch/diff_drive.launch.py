from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        ('wheel_separation', '0.15', 'm between the wheel centers'),
        ('max_wheel_speed', '0.5', 'm/s of a wheel at full duty cycle'),
        ('left_topic', '/tb6612_driver_node/motor_a/cmd_vel', 'Float32 topic of the left motor'),
        ('right_topic', '/tb6612_driver_node/motor_b/cmd_vel', 'Float32 topic of the right motor'),
        ('invert_left', 'false', 'left motor turns the wrong way'),
        ('invert_right', 'false', 'right motor turns the wrong way'),
        ('cmd_vel_topic', 'cmd_vel', 'Twist input (cmd_vel_safe behind cmd_vel_mux_node)'),
    ]
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default, description=desc) for name, default, desc in args],
        Node(
            package='xpi_actuators',
            executable='diff_drive_node',
            name='diff_drive_node',
            output='screen',
            parameters=[{
                'wheel_separation': LaunchConfiguration('wheel_separation'),
                'max_wheel_speed': LaunchConfiguration('max_wheel_speed'),
                'left_topic': LaunchConfiguration('left_topic'),
                'right_topic': LaunchConfiguration('right_topic'),
                'invert_left': LaunchConfiguration('invert_left'),
                'invert_right': LaunchConfiguration('invert_right'),
            }],
            remappings=[('cmd_vel', LaunchConfiguration('cmd_vel_topic'))],
        ),
    ])
