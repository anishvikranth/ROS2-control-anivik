import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rover_aruco'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # FIX: Using os.path.join inside glob ensures files are found during colcon build
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*launch.[pxy][yma]'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anivik',
    maintainer_email='anishvikranth@gmail.com',
    description='Rover ArUco navigation and perception package',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'aruco_detector = rover_aruco.perception.aruco_detector:main',
            'marker_tracker = rover_aruco.navigation.marker_tracker:main',
            'goal_generator = rover_aruco.navigation.goal_generator:main',
            'obs_avoid = rover_aruco.movement:main'
        ],
    },
)