from setuptools import find_packages, setup

package_name = 'wr_swerve_control'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Listens for speeds and anlges of each swerve module and publishes appropriate CAN messages to can_msg',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'swerve_control = wr_swerve_control.swerve_control:main'
        ],
    },
)
