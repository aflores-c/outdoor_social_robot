from setuptools import setup
from glob import glob
import os

package_name = 'wheel_odometry_bringup'

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
        'Optional odom -> base_footprint TF broadcaster sourced from the '
        'mobile base controller wheel odometry, as an alternative to '
        'scan_matcher_bringup.'
    ),
    license='MIT',
    entry_points={
        'console_scripts': [
            'wheel_odom_tf_node = wheel_odometry_bringup.wheel_odom_tf_node:main',
        ],
    },
)
