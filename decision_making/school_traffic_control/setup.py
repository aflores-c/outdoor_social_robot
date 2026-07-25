from setuptools import setup
from glob import glob
import os

package_name = 'school_traffic_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andy Flores',
    maintainer_email='andy.flores@pucp.edu.pe',
    description=(
        'School traffic control decision node: moves the robot arm and base '
        'based on vehicle/pedestrian detections and plate-allowed status.'
    ),
    license='MIT',
    entry_points={
        'console_scripts': [
            'school_traffic_control_node = school_traffic_control.school_traffic_control_node:main',
        ],
    },
)
