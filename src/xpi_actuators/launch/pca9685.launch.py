from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('i2c_bus', default_value='1', description='I2C Bus Number'),
        DeclareLaunchArgument('i2c_address', default_value='0x40', description='I2C Address (Hex as int, e.g. 64)'),
        DeclareLaunchArgument('frequency', default_value='50.0', description='PWM Frequency (Hz)'),
        DeclareLaunchArgument('mock_hardware', default_value='false', description='Mock I2C'),

        Node(
            package='xpi_actuators',
            executable='pca9685_node',
            name='pwm_driver',
            parameters=[{
                'i2c_bus': LaunchConfiguration('i2c_bus'),
                'i2c_address': 64, # 0x40
                'frequency': LaunchConfiguration('frequency'),
                'mock_hardware': LaunchConfiguration('mock_hardware')
            }]
        )
    ])
