import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'state_machine'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'resource'), glob('resource/*.txt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='david-wang',
    maintainer_email='wang3458@wisc.edu',
    description='state machine',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'state_machine = state_machine.state_machine_node:main',
            'state_machine_controller = state_machine.state_machine_controller_node:main'
        ],
    },
)
