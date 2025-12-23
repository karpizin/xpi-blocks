from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'polling_rate',
            default_value='2.0',
            description='Polling rate in Hz'
        ),
        DeclareLaunchArgument(
            'gain',
            default_value='MEDIUM',
            description='Sensor Gain: LOW (1x), MEDIUM (25x), HIGH (428x), MAX (9876x)'
        ),
        DeclareLaunchArgument(
            'integration_time',
            default_value='100MS',
            description='Integration Time: 100MS to 600MS'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='tsl2591_link',
            description='Frame ID for TF'
        ),
        
        Node(
            package='xpi_sensors',
            executable='tsl2591_node',
            name='tsl2591_light_sensor',
            parameters=[{
                'polling_rate': LaunchConfiguration('polling_rate'),
                'gain': LaunchConfiguration('gain'),
                'integration_time': LaunchConfiguration('integration_time'),
                'frame_id': LaunchConfiguration('frame_id')
            }],
            output='screen'
        )
    ])
