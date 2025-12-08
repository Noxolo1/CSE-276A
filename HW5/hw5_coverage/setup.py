# from setuptools import find_packages, setup

# package_name = 'hw5_coverage'

# setup(
#     name=package_name,
#     version='0.0.0',
#     packages=find_packages(exclude=['test']),
#     data_files=[
#         ('share/ament_index/resource_index/packages',
#          ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#         ('share/' + package_name + '/configs',
#          ['configs/apriltags_position.yaml']),
#         ('share/' + package_name + '/launch',
#          ['launch/hw5_coverage.launch.py']),
#     ],
#     install_requires=['setuptools', 'pyyaml'],
#     zip_safe=True,
#     maintainer='nate wilson',
#     maintainer_email='new002@ucsd.edu',
#     description='CSE 276A HW5 coverage controller package',
#     license='MIT',
#     tests_require=['pytest'],
#     entry_points={
#         'console_scripts': [
#             'hw5_coverage_node = hw5_coverage.coverage_node:main',
#             'hw5_velocity_mapping = hw5_coverage.hw5_velocity_mapping:main',
#             'hw5_waypoint_follower = hw5_coverage.hw5_waypoint_follower:main',
#         ],
#     }
# )



from setuptools import setup
import os
from glob import glob

package_name = 'hw5_coverage'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # This part is standard for all ROS2 Python pkgs
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # THIS line installs all *.launch.py from the launch/ folder
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools', 'pyyaml', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='YOUR_NAME',
    maintainer_email='YOUR_EMAIL',
    description='HW5 coverage package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hw5_coverage_node = hw5_coverage.coverage_node:main',
            'hw5_velocity_mapping = hw5_coverage.hw5_velocity_mapping:main',
            'hw5_waypoint_follower = hw5_coverage.hw5_waypoint_follower:main',
        ],
    },
)
