from setuptools import setup

package_name = 'xpi_commons'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Viacheslav Karpizin',
    maintainer_email='viacheslav.karpizin@gmail.com',
    description='Common utilities and HAL for XPI blocks',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'eeprom_node = xpi_commons.eeprom_node:main',
            'w25qxx_node = xpi_commons.w25qxx_node:main',
            'i2c_mux_node = xpi_commons.i2c_mux_node:main',
        ],
    },
)
