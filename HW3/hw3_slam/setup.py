from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'hw3_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'configs'), glob('configs/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='new002@ucsd.edu',
    description='Homework 3 EKF SLAM package',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hw3_slam = hw3_slam.ekf_slam:main',
            'hw3_controller = hw3_slam.hw3_controller:main',
            'hw3_velocity_mapping = hw3_slam.velocity_mapping:main',
        ],
    },
)
