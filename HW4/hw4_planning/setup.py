# # hw4_planning/setup.py
# from setuptools import find_packages, setup

# package_name = 'hw4_planning'

# setup(
#     name=package_name,
#     version='0.0.0',
#     packages=find_packages(exclude=['test']),
#     data_files=[
#         ('share/ament_index/resource_index/packages',
#          ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#         # You can keep a copy of apriltags_position.yaml here if you like.
#         ('share/' + package_name + '/configs', ['configs/apriltags_position.yaml']),
#         ('share/' + package_name + '/launch', ['launch/hw4_planning.launch.py']),
#         ('share/' + package_name + '/launch', ['launch/waypoint_follower_launch.py']),
#     ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='nate wilson',
#     maintainer_email='new002@ucsd.edu',
#     description='CSE 276A HW4 planning package',
#     license='MIT',
#     tests_require=['pytest'],
#     entry_points={
#         'console_scripts': [
#             'hw4_planning_node = hw4_planning.planner_node:main',
#             'hw4_velocity_mapping = hw4_planning.velocity_mapping:main',
#             'waypoint_follower_node = hw4_planning.waypoint_follower_node:main',
#             'hw4_json_waypoint_follower = hw4_planning.json_waypoint_follower:main',
#         ],
#     },
# )

# hw4_planning/setup.py
from setuptools import find_packages, setup

package_name = 'hw4_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # You can keep a copy of apriltags_position.yaml here if you like.
        ('share/' + package_name + '/configs', ['configs/apriltags_position.yaml']),
        ('share/' + package_name + '/launch', ['launch/hw4_planning.launch.py','launch/hw4_astar_planning.launch.py','launch/waypoint_follower_launch.py']),
    ],
    # runtime dependencies installed via ROS package manager (package.xml).
    # Include minimal pip-installable requirements so `python setup.py develop` works
    # in a vanilla environment for offline testing.
    install_requires=['setuptools', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='nate wilson',
    maintainer_email='new002@ucsd.edu',
    description='CSE 276A HW4 planning package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planning_node = hw4_planning.planning_node:main',
            'localization_hw4 = hw4_planning.localization_hw4:main',
            'waypoint_follower_hw4 = hw4_planning.waypoint_follower_hw4:main',
            # velocity mapping node (converts /cmd_vel -> /motor_commands)
            'velocity_mapping = hw4_planning.velocity_mapping:main',
        ],
    },
)