from setuptools import find_packages, setup

package_name = 'wr_xbox_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pygame'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@example.com',
    description='Example Xbox publisher',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'xbox_controller = wr_xbox_controller.xbox_controller:main',
            'old_xbox_controller = wr_xbox_controller.old_xbox_controller:main',
            'arm_xbox_control = wr_xbox_controller.arm_xbox:main',
        ],
    },
)
