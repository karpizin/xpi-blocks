from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'i2c_bus',
            default_value='1',
            description='I2C Bus Number (usually 1 for Raspberry Pi)'
        ),
        DeclareLaunchArgument(
            'i2c_address',
            default_value='0x76', # BME280 common address, can be 0x77
            description='I2C Address of the BME280 sensor'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='1.0',
            description='Rate in Hz to publish sensor readings'
        ),
        DeclareLaunchArgument(
            'osrs_t',
            default_value='2', # 2x oversampling for temperature
            description='Oversampling for temperature (0:skipped, 1:1x, 2:2x, 3:4x, 4:8x, 5:16x)'
        ),
        DeclareLaunchArgument(
            'osrs_p',
            default_value='5', # 16x oversampling for pressure
            description='Oversampling for pressure (0:skipped, 1:1x, 2:2x, 3:4x, 4:8x, 5:16x)'
        ),
        DeclareLaunchArgument(
            'osrs_h',
            default_value='1', # 1x oversampling for humidity
            description='Oversampling for humidity (0:skipped, 1:1x, 2:2x, 3:4x, 4:8x, 5:16x)'
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='3', # Normal mode
            description='Power mode (0:sleep, 1:forced, 3:normal)'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='bme280_link',
            description='Frame ID for sensor_msgs messages'
        ),
        DeclareLaunchArgument(
            'mock_hardware',
            default_value='false',
            description='Run in mock mode without real BME280 hardware'
        ),
        
        Node(
            package='xpi_sensors',
            executable='bme280_node',
            name='bme280_env_sensor',
            parameters=[{
                'i2c_bus': LaunchConfiguration('i2c_bus'),
                'i2c_address': LaunchConfiguration('i2c_address'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'osrs_t': LaunchConfiguration('osrs_t'),
                'osrs_p': LaunchConfiguration('osrs_p'),
                'osrs_h': LaunchConfiguration('osrs_h'),
                'mode': LaunchConfiguration('mode'),
                'frame_id': LaunchConfiguration('frame_id'),
                'mock_hardware': LaunchConfiguration('mock_hardware')
            }],
            output='screen'
        )
    ])
