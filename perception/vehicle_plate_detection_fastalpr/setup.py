from setuptools import setup
from glob import glob
import os

package_name = 'vehicle_plate_detection_fastalpr'

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
        'Vehicle license plate detection + registration check via fast-alpr '
        '(ONNX Runtime), as an alternative to vehicle_plate_detection\'s '
        'YOLO+EasyOCR pipeline for side-by-side testing.'
    ),
    license='MIT',
    entry_points={
        'console_scripts': [
            'plate_detector_fastalpr_node = vehicle_plate_detection_fastalpr.plate_detector_fastalpr_node:main',
        ],
    },
)
