from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gazebo_simulation'

def list_files(src_dir, dst_dir):
    """
    Recursively collect all files under src_dir,
    preserving their relative paths under dst_dir.
    """
    results = []
    for root, dirs, files in os.walk(src_dir):
        for f in files:
            src_path = os.path.join(root, f)                     # full path
            rel_path = os.path.relpath(src_path, src_dir)       # relative path under files/
            dest = os.path.join(dst_dir, os.path.dirname(rel_path))
            results.append((dest, [src_path]))
    return results


# Collect all files under files/
files_data = list_files('files', f'share/{package_name}/files')


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
    ] + files_data,   # <-- add recursive file structure here
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jecerquerac',
    maintainer_email='cerqueracano.j@northeastern.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={},
)
