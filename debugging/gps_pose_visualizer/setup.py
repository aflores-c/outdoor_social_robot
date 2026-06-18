from setuptools import setup
import os
from glob import glob

package_name = 'gps_pose_visualizer'

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
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andy Flores',
    maintainer_email='andy.flores@pucp.edu.pe',
    description='Visualizes GPS position, RTK accuracy, fix type, and robot trajectory in RViz2',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'gps_info_node = gps_pose_visualizer.gps_info_node:main',
        ],
    },
)
