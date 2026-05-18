from setuptools import setup
from glob import glob
import os

package_name = 'semantic_segmentation'

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
    maintainer='cas',
    maintainer_email='andy.flores@pucp.edu.pe',
    description='YOLOv8-seg + SegFormer inference node for outdoor robot perception',
    license='MIT',
    entry_points={
        'console_scripts': [
            'segmentation_node = semantic_segmentation.segmentation_node:main',
            'export_tensorrt   = semantic_segmentation.export_tensorrt:main',
        ],
    },
)
