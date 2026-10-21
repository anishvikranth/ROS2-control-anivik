import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rover_traversability'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'scripts'),
            glob(os.path.join('scripts', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anivik',
    maintainer_email='anishvikranth@gmail.com',
    description='Terrain mapping and traversability for the rover',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'terrain_mapper = rover_traversability.terrain_mapper:main',
        ],
    },
)
