from setuptools import setup
import os
from glob import glob

package_name = 'xpi_llm'

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
    description='LLM/VLM integration for sensor analysis and smart control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sonar_trend_analyzer_node = xpi_llm.sonar_trend_analyzer_node:main',
            'tool_calling_node = xpi_llm.tool_calling_node:main',
            'sonar_pattern_analyzer_node = xpi_llm.sonar_pattern_analyzer_node:main',
            'vlm_observer_node = xpi_llm.vlm_observer_node:main',
            'audio_analyzer_node = xpi_llm.audio_analyzer_node:main',
        ],
    },
)
