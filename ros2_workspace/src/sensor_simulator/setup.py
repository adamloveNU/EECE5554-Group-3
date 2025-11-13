from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sensor_simulator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
        glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'data'), 
        glob('data/*.txt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jecerquerac',
    maintainer_email='cerqueracano.j@northeastern.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'imu_simulator = sensor_simulator.imu_simulator:main',
        ],
    },
)
