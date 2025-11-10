########### version 2 setup.py


# from setuptools import find_packages, setup

# package_name = 'hw3_slam'

# setup(
#     name=package_name,
#     version='0.0.0',
#     packages=find_packages(exclude=['test']),
#     data_files=[
#         ('share/ament_index/resource_index/packages',
#             ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#         # Add configs later if you create param files for HW3
#         ('share/' + package_name + '/configs', []),
#         ('share/' + package_name + '/launch', ['launch/hw3_slam.launch.py']),
#     ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='root',
#     maintainer_email='new002@ucsd.edu',
#     description='Homework 3 EKF-SLAM package',
#     license='MIT',
#     extras_require={
#         'test': [
#             'pytest',
#         ],
#     },
#     entry_points={
#         'console_scripts': [
#             # main EKF-SLAM node
#             'hw3_slam = hw3_slam.ekf_slam:main',
#         ],
#     },
# )


########### version 3 setup.py

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
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Include config files if any (for future use)
        (os.path.join('share', package_name, 'configs'), glob('configs/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='new002@ucsd.edu',
    description='Homework 3 EKF-SLAM package',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # Main EKF-SLAM node
            'hw3_slam = hw3_slam.ekf_slam:main',
            # Waypoint controller using SLAM pose
            'hw3_controller = hw3_slam.hw3_controller:main',
            'hw3_velocity_mapping = hw3_slam.velocity_mapping:main',
        ],
    },
)
