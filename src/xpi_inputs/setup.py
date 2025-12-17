from setuptools import setup
import os
from glob import glob

package_name = 'xpi_inputs'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'web_static'), glob('web_static/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='XPI Maintainer',
    maintainer_email='user@example.com',
    description='Input device blocks for XPI',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sbus_receiver_node = xpi_inputs.sbus_receiver_node:main',
            'crsf_receiver_node = xpi_inputs.crsf_receiver_node:main',
            'ppm_receiver_node = xpi_inputs.ppm_receiver_node:main',
            'joy_mapper_node = xpi_inputs.joy_mapper_node:main',
            'keyboard_to_joy_node = xpi_inputs.keyboard_to_joy_node:main',
            'mouse_node = xpi_inputs.mouse_node:main',
            'web_joystick_node = xpi_inputs.web_joystick_node:main',
            'telegram_bot_node = xpi_inputs.telegram_bot_node:main',
        ],
    },
)
