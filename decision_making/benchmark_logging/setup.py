from setuptools import setup
from glob import glob
import os

package_name = 'benchmark_logging'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andy Flores',
    maintainer_email='andy.flores@pucp.edu.pe',
    description=(
        'Field-trial data-collection instrumentation: trial management '
        'and JSONL event/detection/pose logging for school_traffic_control.'
    ),
    license='MIT',
    entry_points={
        'console_scripts': [
            'trial_manager_node = benchmark_logging.trial_manager_node:main',
            'data_logger_node = benchmark_logging.data_logger_node:main',
            'start_trial = benchmark_logging.cli:start_trial',
            'stop_trial = benchmark_logging.cli:stop_trial',
        ],
    },
)
