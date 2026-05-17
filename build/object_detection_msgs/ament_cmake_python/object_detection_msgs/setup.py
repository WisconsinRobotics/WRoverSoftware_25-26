from setuptools import find_packages
from setuptools import setup

setup(
    name='object_detection_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('object_detection_msgs', 'object_detection_msgs.*')),
)
