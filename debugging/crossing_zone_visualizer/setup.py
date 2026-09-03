from setuptools import setup
from glob import glob
import os

package_name = 'crossing_zone_visualizer'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andy Flores',
    maintainer_email='andy.flores@pucp.edu.pe',
    description='Visualizes crossing_zone_monitor\'s fixed crossing-lane zone and live occupancy state in RViz2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'crossing_zone_viz_node = crossing_zone_visualizer.crossing_zone_viz_node:main',
        ],
    },
)
