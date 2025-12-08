from setuptools import setup
import os
from glob import glob

package_name = 'hw5_coverage'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),

        (os.path.join('share', package_name, 'configs'),
         glob('configs/*.yaml')),
    ],
    install_requires=['setuptools', 'pyyaml', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='nate wilson',
    maintainer_email='new002@ucsd.edu',
    description='CSE 276A HW5 coverage / Roomba-style controller',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hw5_velocity_mapping = hw5_coverage.velocity_mapping:main',
            'hw5_waypoint_coverage = hw5_coverage.waypoint_coverage:main',
            'hw5_waypoint_path_planner = hw5_coverage.waypoint_path_planner:main',
        ],
    },
)
