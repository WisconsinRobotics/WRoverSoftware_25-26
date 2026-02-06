from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'object_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'resource'), glob('resource/*.blob')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='david-wang',
    maintainer_email='wang3458@wisc.edu',
    description='object detection with yolov8',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'bottle = object_detection.bottle:main',
            'mallet = object_detection.mallet:main',
            'hammer = object_detection.hammer:main', 
        ],
    },
)
