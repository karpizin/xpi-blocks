from setuptools import setup
import os
from glob import glob

package_name = 'xpi_comms'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='XPI Maintainer',
    maintainer_email='user@example.com',
    description='Communication bridges for XPI blocks',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_bridge = xpi_comms.serial_bridge_node:main',
            'modbus_rtu_node = xpi_comms.modbus_rtu_node:main',
            'mqtt_bridge_node = xpi_comms.mqtt_bridge_node:main',
            'mqtt_gateway_node = xpi_comms.mqtt_gateway_node:main',
            'can_bridge_node = xpi_comms.can_bridge_node:main',
            'meshtastic_bridge = xpi_comms.meshtastic_bridge_node:main',
            'swarm_controller_node = xpi_comms.swarm_controller_node:main',
            'ble_bridge_node = xpi_comms.ble_bridge_node:main',
            'lte_modem_node = xpi_comms.lte_modem_node:main',
            'rtc_monitor_node = xpi_comms.rtc_monitor_node:main',
            'hc12_node = xpi_comms.hc12_node:main',
            'lora_raw_node = xpi_comms.lora_raw_node:main',
        ],
    },
)