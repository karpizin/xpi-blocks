from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'i2c_address',
            default_value='0x77', # BME680 default for Adafruit is 0x77
            description='I2C Address of the BME680 sensor'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='1.0',
            description='Rate in Hz to publish sensor readings'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='bme680_link',
            description='Frame ID for sensor_msgs messages'
        ),
        DeclareLaunchArgument(
            'sea_level_pressure',
            default_value='1013.25',
            description='Sea level pressure in hPa for altitude calculation (internal)'
        ),
        
        Node(
            package='xpi_sensors',
            executable='bme680_node',
            name='bme680_env_sensor',
            parameters=[{
                'i2c_address': LaunchConfiguration('i2c_address'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'frame_id': LaunchConfiguration('frame_id'),
                'sea_level_pressure': LaunchConfiguration('sea_level_pressure')
            }],
            output='screen'
        )
    ])
