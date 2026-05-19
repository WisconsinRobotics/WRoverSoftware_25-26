from setuptools import find_packages, setup

package_name = 'master_finder'

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
    maintainer='david-wang',
    maintainer_email='wang3458@wisc.edu',
    description='master finder for spiral path',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
			'master_finder = master_finder.master_finder:main'
        ],
    },
)
