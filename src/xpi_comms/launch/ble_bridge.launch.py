from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'device_name',
            default_value='XPI-Robot',
            description='Name advertised over Bluetooth'
        ),
        
        Node(
            package='xpi_comms',
            executable='ble_bridge_node',
            name='ble_interface',
            parameters=[{
                'device_name': LaunchConfiguration('device_name')
            }],
            output='screen'
        )
    ])
