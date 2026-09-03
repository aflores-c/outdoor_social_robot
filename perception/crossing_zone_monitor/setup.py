from setuptools import setup
from glob import glob
import os

package_name = 'crossing_zone_monitor'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andy Flores',
    maintainer_email='andy.flores@pucp.edu.pe',
    description=(
        'Ground-filtered Velodyne point-in-zone occupancy check for the '
        'vehicle-crossing lane, independent of the camera-based classifier.'
    ),
    license='MIT',
    entry_points={
        'console_scripts': [
            'crossing_zone_monitor_node = crossing_zone_monitor.crossing_zone_monitor_node:main',
        ],
    },
)
