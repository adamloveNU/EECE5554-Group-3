from setuptools import setup
import os
from glob import glob

package_name = 'slam_visualization'

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
        # Include RViz configs
        (os.path.join('share', package_name, 'rviz'), 
            glob('rviz/*.rviz')),
        # Include PlotJuggler configs
        (os.path.join('share', package_name, 'plotjuggler'), 
            glob('plotjuggler/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Juan',
    maintainer_email='you@example.com',
    description='Visualization tools for SLAM robot (RViz, PlotJuggler, etc.)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
        ],
    },
)