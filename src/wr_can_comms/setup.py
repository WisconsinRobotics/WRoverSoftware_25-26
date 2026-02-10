from setuptools import find_packages, setup

package_name = 'wr_can_comms'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'python-can'],
    zip_safe=True,
    maintainer='aren',
    maintainer_email='aren@example.com',
    description='Receives CAN message requests and sends them',
    license='Apache-2.0',
    tests_require=[],
    entry_points={
        'console_scripts': [
            'can_comms = wr_can_comms.can_comms:main'
        ],
    },
)
