from setuptools import setup
import os
from glob import glob

package_name = 'osm_gps_visualizer'

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
    description='Live GPS tracker on OpenStreetMap via a Flask web server',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'osm_node = osm_gps_visualizer.osm_node:main',
        ],
    },
)
