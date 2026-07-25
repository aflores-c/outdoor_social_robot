from setuptools import setup
from glob import glob
import os

package_name = 'traffic_object_detection'

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
        'Combined pedestrian + vehicle detection via a single YOLO pass '
        '+ LiDAR back-projection.'
    ),
    license='MIT',
    entry_points={
        'console_scripts': [
            'traffic_object_detector_node = traffic_object_detection.traffic_object_detector_node:main',
        ],
    },
)
