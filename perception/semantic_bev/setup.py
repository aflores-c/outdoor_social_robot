from setuptools import setup
from glob import glob
import os

package_name = 'semantic_bev'

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
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cas',
    maintainer_email='andy.flores@pucp.edu.pe',
    description='Local semantic BEV map fusing LiDAR + FAST-LIO + RGB-D/YOLO',
    license='MIT',
    entry_points={
        'console_scripts': [
            'semantic_bev_node = semantic_bev.semantic_bev_node:main',
        ],
    },
)
