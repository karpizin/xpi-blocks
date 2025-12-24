from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'input_voltage_topic',
            default_value='/ads1115/voltage_ch0',
            description='Topic where voltage from ADC is published'
        ),
        Node(
            package='xpi_sensors',
            executable='wind_vane_node',
            name='wind_vane',
            namespace='wind_vane',
            parameters=[{
                'input_voltage_topic': LaunchConfiguration('input_voltage_topic'),
            }],
            output='screen'
        )
    ])
