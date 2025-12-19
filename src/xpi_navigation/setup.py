from setuptools import setup
import os
from glob import glob

package_name = 'xpi_navigation'

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
    install_requires=['setuptools', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='XPI Maintainer',
    maintainer_email='user@example.com',
    description='Indoor navigation and SLAM blocks for XPI',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uwb_ranging_node = xpi_navigation.uwb_ranging_node:main',
            'beacon_slam_node = xpi_navigation.beacon_slam_node:main',
        ],
    },
)
