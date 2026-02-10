import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'wr_arm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nico',
    maintainer_email='nicolasdittmarg1@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'position_control = wr_arm.position_control:main',
            'velocity_control = wr_arm.velocity_control:main',
            'send_to_can = wr_arm.send_to_can:main',
        ],
    },
)
