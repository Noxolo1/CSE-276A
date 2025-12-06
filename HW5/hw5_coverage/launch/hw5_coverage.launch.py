from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """
    Start nodes in sequence for HW5 coverage:
    1. robot_vision_camera
    2. apriltag_ros
    3. after 5s: motor_control + hw4_velocity_mapping
    4. after 10s: hw5_coverage_node
    """

    # Camera pipeline
    camera_pkg_path = FindPackageShare('robot_vision_camera').find('robot_vision_camera')
    camera_launch_file = os.path.join(
        camera_pkg_path, 'launch', 'robot_vision_camera.launch.py'
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch_file)
    )

    # AprilTag detection
    apriltag_pkg_path = FindPackageShare('apriltag_ros').find('apriltag_ros')
    apriltag_launch_file = os.path.join(
        apriltag_pkg_path, 'launch', 'apriltag_launch.py'
    )

    apriltag_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(apriltag_launch_file)
    )

    # Motor controller on the RubikPi (same as previous HWs)
    motor_controller = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='robot_control',
                executable='motor_control',
                name='motor_control',
                output='screen',
                emulate_tty=True,
            )
        ]
    )

    # Reuse your HW4 velocity mapping node
    # velocity_mapping = TimerAction(
    #     period=5.0,
    #     actions=[
    #         Node(
    #             package='hw4_planning',
    #             executable='hw4_velocity_mapping',
    #             name='hw4_velocity_mapping',
    #             output='screen',
    #             emulate_tty=True,
    #         )
    #     ]
    # )
    velocity_mapping = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='hw5_coverage',
                executable='hw5_velocity_mapping',
                name='hw5_velocity_mapping',
                output='screen',
                emulate_tty=True,
                parameters=[
                    # You can override any of the defaults here if you want:
                    # {'left_linear_deadzone': 0.13},
                    # {'left_linear_slope': 3.5},
                    # ...
                ],
            )
        ]
    )


    # HW5 coverage controller
    coverage_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='hw5_coverage',
                executable='hw5_coverage_node',
                name='hw5_coverage_node',
                output='screen',
                emulate_tty=True,
                parameters=[
                    # where to log trajectories (used later in your report)
                    {'trajectory_log_file': 'hw5_coverage_trajectory.json'},
                    # tune these if needed
                    {'explore_speed': 0.15},
                    {'boundary_margin': 0.15},
                    {'lost_pose_timeout': 2.0},
                ],
            )
        ]
    )

    return LaunchDescription([
        camera_launch,
        apriltag_launch,
        motor_controller,
        velocity_mapping,
        coverage_node,
    ])
