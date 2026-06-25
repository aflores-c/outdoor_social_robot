from setuptools import setup
import os
from glob import glob

package_name = 'lidar_camera_calibration'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'results'),
            glob('results/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andy Flores',
    maintainer_email='andy.flores@pucp.edu.pe',
    description='Extrinsic calibration: Velodyne VLP-32C ↔ Intel RealSense D455 (ChArUco + plane-SVD)',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'collect_samples_node = lidar_camera_calibration.collect_samples_node:main',
            'estimate_transform = lidar_camera_calibration.estimate_transform:main',
            'validate_projection_node = lidar_camera_calibration.validate_projection_node:main',
        ],
    },
)
