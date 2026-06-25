from setuptools import setup
from glob import glob
import os

package_name = 'pedestrian_detection'

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
    description='Pedestrian detection + 3D pose estimation via YOLO + LiDAR back-projection',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pedestrian_detector_node = pedestrian_detection.pedestrian_detector_node:main',
        ],
    },
)
