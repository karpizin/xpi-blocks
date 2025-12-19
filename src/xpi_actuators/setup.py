from setuptools import setup
import os
from glob import glob

package_name = 'xpi_actuators'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='XPI Maintainer',
    maintainer_email='user@example.com',
    description='Output device blocks for XPI',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'led_bar = xpi_actuators.led_bar_node:main',
            'led_matrix = xpi_actuators.led_matrix_node:main',
            'pca9685_node = xpi_actuators.pca9685_node:main',
            'relay_node = xpi_actuators.relay_node:main',
            'ssd1306_node = xpi_actuators.ssd1306_node:main',
            'tm1637 = xpi_actuators.tm1637_node:main',
            'unipolar_stepper = xpi_actuators.unipolar_stepper_node:main',
            'ws2812_driver = xpi_actuators.ws2812_driver_node:main',
            'tb6612_driver = xpi_actuators.tb6612_driver_node:main',
            'l298_driver = xpi_actuators.l298_driver_node:main',
            'a4988_driver = xpi_actuators.a4988_driver_node:main',
            'esc_driver = xpi_actuators.esc_driver_node:main',
            'direct_servo = xpi_actuators.direct_servo_node:main',
            'lcd1602_node = xpi_actuators.lcd1602_node:main',
        ],
    },
)
