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
            default_value='0x68', # MPU6050 default address
            description='I2C Address of the MPU6050 sensor'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='50.0',
            description='Rate in Hz to publish IMU and temperature readings'
        ),
        DeclareLaunchArgument(
            'accel_fsr',
            default_value='0', # 0=2g, 1=4g, 2=8g, 3=16g
            description='Accelerometer Full Scale Range (0: +/-2g, 1: +/-4g, 2: +/-8g, 3: +/-16g)'
        ),
        DeclareLaunchArgument(
            'gyro_fsr',
            default_value='0', # 0=250dps, 1=500dps, 2=1000dps, 3=2000dps
            description='Gyroscope Full Scale Range (0: +/-250dps, 1: +/-500dps, 2: +/-1000dps, 3: +/-2000dps)'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='imu_link',
            description='Frame ID for the sensor_msgs/Imu and sensor_msgs/Temperature messages'
        ),
        DeclareLaunchArgument(
            'mock_hardware',
            default_value='false',
            description='Run in mock mode without real MPU6050 hardware'
        ),
        
        Node(
            package='xpi_sensors',
            executable='mpu6050_node',
            name='mpu6050_sensor',
            parameters=[{
                'i2c_bus': LaunchConfiguration('i2c_bus'),
                'i2c_address': LaunchConfiguration('i2c_address'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'accel_fsr': LaunchConfiguration('accel_fsr'),
                'gyro_fsr': LaunchConfiguration('gyro_fsr'),
                'frame_id': LaunchConfiguration('frame_id'),
                'mock_hardware': LaunchConfiguration('mock_hardware')
            }],
            output='screen'
        )
    ])
