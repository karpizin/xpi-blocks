"""Two-wheeled rover on a TB6612: motor driver + diff drive + cmd_vel mux with emergency stop.

    ros2 launch xpi_projects rover_tb6612.launch.py web:=true estop_gpio:=21

Every velocity source publishes to its own mux input (cmd_vel/web, cmd_vel/rc, ...); only
the mux output cmd_vel_safe reaches the wheels. A node that still publishes plain /cmd_vel
moves nothing: remap it to one of the mux inputs (see blocks/actuators/cmd_vel_mux).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument('wheel_separation', default_value='0.15', description='m between the wheels'),
        DeclareLaunchArgument('max_wheel_speed', default_value='0.5', description='m/s of a wheel at full duty'),
        DeclareLaunchArgument('estop_gpio', default_value='-1',
                              description='BCM pin of a normally-closed stop button to GND, -1 = none'),
        DeclareLaunchArgument('web', default_value='false', description='also start the web joystick'),
        DeclareLaunchArgument('mock_hardware', default_value='false', description='simulated GPIO'),

        Node(package='xpi_actuators', executable='tb6612_driver_node', name='tb6612_driver_node',
             output='screen', parameters=[{'mock_hardware': arg('mock_hardware')}]),
        Node(package='xpi_actuators', executable='diff_drive_node', name='diff_drive_node',
             output='screen',
             parameters=[{'wheel_separation': arg('wheel_separation'),
                          'max_wheel_speed': arg('max_wheel_speed')}],
             remappings=[('cmd_vel', 'cmd_vel_safe')]),
        Node(package='xpi_actuators', executable='cmd_vel_mux_node', name='cmd_vel_mux_node',
             output='screen',
             parameters=[{'estop_gpio': arg('estop_gpio'), 'mock_hardware': arg('mock_hardware')}]),
        Node(package='xpi_inputs', executable='web_joystick_node', name='web_joy',
             output='screen', condition=IfCondition(arg('web')),
             remappings=[('cmd_vel', 'cmd_vel/web')]),
    ])
