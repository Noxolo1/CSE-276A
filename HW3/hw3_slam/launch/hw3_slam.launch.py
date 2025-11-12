#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """
    Launch file for hw3_slam package.
    Starts nodes in sequence:
    1. robot_vision_camera - camera driver
    2. apriltag_ros - AprilTag detection
    3. [5s delay]
    4. motor_control - communicates with robot hardware via serial
    5. velocity_mapping - converts Twist commands to motor commands
    6. camera_tf - static tf node 
    7. hw3_slam - EKF SLAM node (main SLAM implementation)
    """

    camera_pkg_path = FindPackageShare(package='robot_vision_camera').find('robot_vision_camera')
    apriltag_pkg_path = FindPackageShare(package='apriltag_ros').find('apriltag_ros')

    camera_launch_file = os.path.join(camera_pkg_path, 'launch', 'robot_vision_camera.launch.py')
    apriltag_launch_file = os.path.join(apriltag_pkg_path, 'launch', 'apriltag_launch.py')

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch_file)
    )

    apriltag_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(apriltag_launch_file)
    )

    motor_controller = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='hw_2_solution',
                executable='motor_control',
                name='motor_control',
                output='screen',
                emulate_tty=True,
            )
        ]
    )

    velocity_mapping = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='hw3_slam',
                executable='hw3_velocity_mapping',
                name='hw3_velocity_mapping',
                output='screen',
                emulate_tty=True,
            )
        ]
    )

    # reusing from hw2
    camera_tf = Node(
        package='hw_2_solution',
        executable='camera_tf',
        name='camera_tf',
        output='screen',
        emulate_tty=True,
    )

    hw3_slam = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='hw3_slam',
                executable='hw3_slam',
                name='hw3_slam_node',
                output='screen',
                emulate_tty=True,
                parameters=[{
                    'dt': 0.05,
                    'log_data': True,
                    'log_dir': '/tmp/hw3_slam_logs'
                }]
            )
        ]
    )

    return LaunchDescription([
        camera_launch,
        apriltag_launch,
        motor_controller,
        velocity_mapping,
        camera_tf,
        hw3_slam,
    ])