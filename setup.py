from glob import glob
import os

from setuptools import setup

package_name = 'ik_pressure_sensor'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'udev'), glob('udev/*.rules')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Cillo-x',
    maintainer_email='58360548+Cillo-x@users.noreply.github.com',
    description=(
        'ROS 2 driver for the IKMSIK RS485 Modbus-RTU barometric pressure transmitter.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pressure_node = ik_pressure_sensor.pressure_node:main',
        ],
    },
)
