import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'kitti_ros2_player'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='gaurav',
    maintainer_email='you@example.com',
    description='ROS 2 player for the KITTI Raw dataset publishing PointCloud2 and Imu for FAST-LIO',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kitti_player = kitti_ros2_player.kitti_player:main',
        ],
    },
)
