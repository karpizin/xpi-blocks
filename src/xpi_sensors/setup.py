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
            'dht_node = xpi_sensors.dht_node:main',
            'gpio_digital_input_node = xpi_sensors.gpio_digital_input_node:main',
            'mpu6050_node = xpi_sensors.mpu6050_node:main',
            'bme280_node = xpi_sensors.bme280_node:main',
            'bme680_node = xpi_sensors.bme680_node:main',
            'bmp085_node = xpi_sensors.bmp085_node:main',
            'ads1115_node = xpi_sensors.ads1115_node:main',
            'anemometer_node = xpi_sensors.anemometer_node:main',
            'analog_sensor_interpreter_node = xpi_sensors.analog_sensor_interpreter_node:main',
            'lm75a_node = xpi_sensors.lm75a_node:main',
            'bh1750_node = xpi_sensors.bh1750_node:main',
            'opt3001_node = xpi_sensors.opt3001_node:main',
            'tsl2561_node = xpi_sensors.tsl2561_node:main',
            'tsl2591_node = xpi_sensors.tsl2591_node:main',
            'tcs34725_node = xpi_sensors.tcs34725_node:main',
            'pms5003_node = xpi_sensors.pms5003_node:main',
            'apds9960_node = xpi_sensors.apds9960_node:main',
            'as7341_node = xpi_sensors.as7341_node:main',
            'max44009_node = xpi_sensors.max44009_node:main',
            'ccs811_node = xpi_sensors.ccs811_node:main',
            'ina219_node = xpi_sensors.ina219_node:main',
            'max17048_node = xpi_sensors.max17048_node:main',
            'gps_node = xpi_sensors.gps_node:main',
            'qmc5883l_node = xpi_sensors.qmc5883l_node:main',
        ],
    },
)
