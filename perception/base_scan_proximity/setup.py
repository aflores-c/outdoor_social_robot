from setuptools import setup
from glob import glob
import os

package_name = 'base_scan_proximity'

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
        'Close-range obstacle-proximity safety net using the base\'s own '
        '2D lidar (/scan), separate from the roof-mounted Velodyne.'
    ),
    license='MIT',
    entry_points={
        'console_scripts': [
            'base_scan_proximity_node = base_scan_proximity.base_scan_proximity_node:main',
        ],
    },
)
