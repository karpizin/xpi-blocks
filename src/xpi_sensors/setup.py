from setuptools import setup
import os
from glob import glob

package_name = 'xpi_sensors'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='XPI Maintainer',
    maintainer_email='user@example.com',
    description='Sensor blocks for XPI',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sonar_node = xpi_sensors.sonar_node:main',
            'ds18b20_node = xpi_sensors.ds18b20_node:main',
            'gpio_digital_input_node = xpi_sensors.gpio_digital_input_node:main',
            'mpu6050_node = xpi_sensors.mpu6050_node:main',
            'bme280_node = xpi_sensors.bme280_node:main',
            'bmp085_node = xpi_sensors.bmp085_node:main',
            'ads1115_node = xpi_sensors.ads1115_node:main',
            'analog_sensor_interpreter_node = xpi_sensors.analog_sensor_interpreter_node:main',
            'lm75a_node = xpi_sensors.lm75a_node:main',
        ],
    },
)
