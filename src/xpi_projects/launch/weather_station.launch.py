from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. BME280
        Node(
            package='xpi_sensors',
            executable='bme280_node',
            name='weather_meteo',
            parameters=[{'i2c_address': 0x76}]
        ),
        
        # 2. CCS811
        Node(
            package='xpi_sensors',
            executable='ccs811_node',
            name='weather_air',
            parameters=[{'polling_rate': 1.0}]
        ),
        
        # 3. MAX44009
        Node(
            package='xpi_sensors',
            executable='max44009_node',
            name='weather_light',
            parameters=[{'polling_rate': 1.0}]
        )
    ])
