from setuptools import setup

package_name = 'xpi_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'textual'],
    zip_safe=True,
    maintainer='XPI Maintainer',
    maintainer_email='user@example.com',
    description='CLI Tools for XPI Blocks (Monitor, Debugger)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'xpi-top = xpi_tools.xpi_top:main',
        ],
    },
)
