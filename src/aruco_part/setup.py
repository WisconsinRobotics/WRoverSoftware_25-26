from setuptools import find_packages, setup

package_name = 'aruco_part'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rodrigo',
    maintainer_email='rochisigo@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'driving_logic = aruco_part.driving_logic:main',
            'fake_tag = aruco_part.fake_tag:main',
            'fake_sensors = aruco_part.fake_sensors:main',
            'path_visualizer = aruco_part.path_visualizer:main',
            'camera_info = aruco_part.camera_info:main',
            'finding_tag = aruco_part.finding_tag:main',
            'combined = aruco_part.combined:main',
            'combined_2 = aruco_part.combined_2:main',
            'keyboard = aruco_part.keyboard:main',
        ],
    },
)
