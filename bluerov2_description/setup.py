from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bluerov2_description'

def package_files(directory):
    """
    Returns a list of (destination, [files]) tuples for setuptools
    installing all nested files inside a directory.
    """
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        if filenames:  # only install non-empty directories
            files = [os.path.join(path, f) for f in filenames]
            install_path = os.path.join('share', package_name, path)
            paths.append((install_path, files))
    return paths

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # required for ROS package indexing
        ('share/ament_index/resource_index/packages',
            [os.path.join('resource', package_name)]),

        # package manifest
        ('share/' + package_name, ['package.xml']),

        # launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ]
    + package_files('resources')  # recursively install all resources
    + package_files('meshes')     # install meshes if you add any later
    + package_files('urdf'),      # install all URDF/xacro files

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mahmoud',
    maintainer_email='mahmoud.tarek.aboelrayat@gmail.com',
    description='BlueROV2 description package',
    license='TODO: License declaration',
)
