from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='xpi_sensors',
            executable='qmc5883l_node.py',
            name='magnetometer',
            output='screen',
            parameters=[{
                'i2c_bus': 1,
                'polling_rate': 10.0,
                'declination': 0.0 # Change this based on your location
            }]
        )
    ])
