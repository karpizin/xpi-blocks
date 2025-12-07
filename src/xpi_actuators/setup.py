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
            'relay_node = xpi_actuators.relay_node:main',
            'pca9685_node = xpi_actuators.pca9685_node:main',
            'tb6612_driver_node = xpi_actuators.tb6612_driver_node:main',
            'ws2812_driver_node = xpi_actuators.ws2812_driver_node:main',
        ],
    },
)
