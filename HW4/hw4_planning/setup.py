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
        ('share/' + package_name + '/configs', ['configs/apriltags_position.yaml']),
        ('share/' + package_name + '/launch', ['launch/hw4_planning.launch.py', 'launch/waypoint_follower_launch.py']),
    ],
    
    install_requires=['setuptools', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='nate wilson',
    maintainer_email='new002@ucsd.edu',
    description='CSE 276A HW4 planning package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hw4_planning_node = hw4_planning.planner_node:main',
            'hw4_velocity_mapping = hw4_planning.velocity_mapping:main',
            'waypoint_follower = hw4_planning.waypoint_follower:main',
        ],
    }
)