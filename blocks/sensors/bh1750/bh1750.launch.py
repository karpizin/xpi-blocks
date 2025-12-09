from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='xpi_sensors',
            executable='bh1750_node',
            name='bh1750',
            output='screen',
            parameters=[
                {'i2c_bus': 1},
                {'i2c_address': 0x23}, # 0x23 (default) or 0x5C
                {'publish_rate': 2.0}, # Hz
                {'frame_id': 'bh1750_link'},
                {'mode': 'CONTINUOUS_HIGH_RES_MODE'}
            ]
        )
    ])
