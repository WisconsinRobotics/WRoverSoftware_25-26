from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'simulation'

def get_data_files(dir):
    data_files = []
    for root, dirs, files in os.walk(dir):
        if files:
            install_dir = os.path.join("share", package_name, root)
            file_paths = [os.path.join(root, f) for f in files]
            data_files.append((install_dir, file_paths))
    return data_files



setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        *get_data_files("models"),
        ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Christopher Teggatz Jr.',
    maintainer_email='categgatzjr@gmail.com',
    description='A package for simulating the robot in Gazebo.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
