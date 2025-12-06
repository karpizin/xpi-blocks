from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('trigger_pin', default_value='23', description='GPIO Trigger Pin'),
        DeclareLaunchArgument('echo_pin', default_value='24', description='GPIO Echo Pin'),
        DeclareLaunchArgument('frame_id', default_value='sonar_link', description='TF Frame ID'),
        DeclareLaunchArgument('mock_hardware', default_value='false', description='Mock mode'),
        
        Node(
            package='xpi_sensors',
            executable='sonar_node',
            name='sonar_front',
            parameters=[{
                'trigger_pin': LaunchConfiguration('trigger_pin'),
                'echo_pin': LaunchConfiguration('echo_pin'),
                'frame_id': LaunchConfiguration('frame_id'),
                'mock_hardware': LaunchConfiguration('mock_hardware')
            }]
        )
    ])
