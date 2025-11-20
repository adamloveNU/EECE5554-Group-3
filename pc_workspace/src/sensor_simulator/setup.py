from setuptools import setup
import os
from glob import glob

package_name = 'sensor_simulator'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),
        # Include data files
        (os.path.join('share', package_name, 'data'), 
            glob('data/*.txt')),
        # Include camera data (videos)
        (os.path.join('share', package_name, 'data/camera'), 
            glob('data/camera/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Juan',
    maintainer_email='you@example.com',
    description='Sensor simulators for SLAM robot testing',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'imu_simulator = sensor_simulator.imu_simulator:main',
            'camera_simulator = sensor_simulator.camera_simulator:main',
            'lidar_simulator = sensor_simulator.lidar_simulator:main',
        ],
    },
)