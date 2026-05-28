from setuptools import find_packages, setup

package_name = 'gps_tools'

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
    maintainer='aarav',
    maintainer_email='2005.aarav.agrawal@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'local_grapher = gps_tools.local_grapher:main',
            'heading = gps_tools.heading:main',
            'single_heading = gps_tools.single_heading:main',
        ],
    },
)
