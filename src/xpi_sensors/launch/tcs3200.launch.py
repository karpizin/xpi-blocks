from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('s0_pin', default_value='23'),
        DeclareLaunchArgument('s1_pin', default_value='24'),
        DeclareLaunchArgument('s2_pin', default_value='25'),
        DeclareLaunchArgument('s3_pin', default_value='8'),
        DeclareLaunchArgument('out_pin', default_value='7'),
        DeclareLaunchArgument('led_pin', default_value='12'),
        DeclareLaunchArgument('mock_hardware', default_value='false'),

        Node(
            package='xpi_sensors',
            executable='tcs3200_node',
            name='tcs3200',
            namespace='tcs3200',
            parameters=[{
                's0_pin': LaunchConfiguration('s0_pin'),
                's1_pin': LaunchConfiguration('s1_pin'),
                's2_pin': LaunchConfiguration('s2_pin'),
                's3_pin': LaunchConfiguration('s3_pin'),
                'out_pin': LaunchConfiguration('out_pin'),
                'led_pin': LaunchConfiguration('led_pin'),
                'mock_hardware': LaunchConfiguration('mock_hardware'),
            }],
            output='screen'
        )
    ])
