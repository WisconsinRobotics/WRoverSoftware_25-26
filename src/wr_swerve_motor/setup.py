from setuptools import find_packages, setup

package_name = 'wr_swerve_motor'

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
    description='Listens for controller input and sends angles and speeds for each swerve module on their own individual topics',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'swerve_motor = wr_swerve_motor.swerve_motor:main'
        ],
    },
)
