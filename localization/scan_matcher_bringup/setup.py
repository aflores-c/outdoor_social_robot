from setuptools import setup
from glob import glob
import os

package_name = 'scan_matcher_bringup'

setup(
    name=package_name,
    version='0.0.0',
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
        'Bringup for ros2_laser_scan_matcher against the VLP-32C-derived '
        '/scan_outdoor, publishing odom -> base_footprint for '
        'amcl_2d_localization.'
    ),
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)
