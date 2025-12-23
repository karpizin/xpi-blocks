from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('display_type', default_value='ST7789'),
        DeclareLaunchArgument('width', default_value='240'),
        DeclareLaunchArgument('height', default_value='240'),
        DeclareLaunchArgument('rotation', default_value='90'),
        
        Node(
            package='xpi_actuators',
            executable='tft_display_node',
            name='tft_display',
            parameters=[{
                'display_type': LaunchConfiguration('display_type'),
                'width': LaunchConfiguration('width'),
                'height': LaunchConfiguration('height'),
                'rotation': LaunchConfiguration('rotation'),
                'cs_pin': 8,
                'dc_pin': 25,
                'rst_pin': 27,
                'bl_pin': 24
            }],
            output='screen'
        )
    ])
