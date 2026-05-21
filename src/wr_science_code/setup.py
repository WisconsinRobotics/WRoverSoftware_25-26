from setuptools import find_packages, setup
import os
from glob import glob

<<<<<<<< HEAD:src/wr_science_data/setup.py
package_name = 'wr_science_data'
========
package_name = 'wr_science_code'
>>>>>>>> d1663b369896f7fe18f43f3d4d0340b10367e37a:src/wr_science_code/setup.py

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
    maintainer='wiscrobo',
<<<<<<<< HEAD:src/wr_science_data/setup.py
    maintainer_email='nicolasdittmarg1@gmail.com',
========
    maintainer_email='devansh.the.photofreak@gmail.com',
>>>>>>>> d1663b369896f7fe18f43f3d4d0340b10367e37a:src/wr_science_code/setup.py
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
<<<<<<<< HEAD:src/wr_science_data/setup.py
            'get_data = wr_science_data.get_data:main',
            'send_to_can = wr_science_data.send_to_can:main',
            'science_control = wr_science_data.science_control:main',
========
            'get_data = wr_science_code.get_data:main',
            'science_control = wr_science_code.science_control:main',
            'send_to_can = wr_science_code.send_to_can:main',
>>>>>>>> d1663b369896f7fe18f43f3d4d0340b10367e37a:src/wr_science_code/setup.py
        ],
    },
)
