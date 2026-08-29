from setuptools import setup
from glob import glob
import os

package_name = 'robot_audio'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'audio'),
            glob('audio/*.mp3')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andy Flores',
    maintainer_email='andy.flores@pucp.edu.pe',
    description=(
        'Robot audio playback node: PlayAudio action server that plays '
        'mp3 files bundled in this package by name.'
    ),
    license='MIT',
    entry_points={
        'console_scripts': [
            'robot_audio_node = robot_audio.robot_audio_node:main',
        ],
    },
)
