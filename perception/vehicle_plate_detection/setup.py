from setuptools import setup
from glob import glob
import os

package_name = 'vehicle_plate_detection'

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
        'Vehicle license plate detection + registration check. Publishes a '
        'bool to school_traffic_control indicating whether a visible plate '
        'is on the allow-list.'
    ),
    license='MIT',
    entry_points={
        'console_scripts': [
            'plate_detector_node = vehicle_plate_detection.plate_detector_node:main',
        ],
    },
)
