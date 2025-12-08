from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration

import os


def generate_launch_description():
    default_wp_path = os.path.join(
        os.path.expanduser("~"),
        "ros2_ws", "rubikpi_ros2", "hw5_coverage", "hw5_coverage",
        "hw5_waypoints_lawnmower.json",
    )

    default_tag_yaml_path = os.path.join(
        os.path.expanduser("~"),
        "ros2_ws", "rubikpi_ros2", "hw5_coverage", "configs",
        "apriltags_position.yaml",
    )

    default_log_dir = os.path.dirname(default_wp_path)

    waypoint_file_arg = DeclareLaunchArgument(
        "waypoint_file",
        default_value=default_wp_path,
        description="Path to HW5 lawnmower waypoint JSON file.",
    )

    tag_yaml_arg = DeclareLaunchArgument(
        "tag_yaml_file",
        default_value=default_tag_yaml_path,
        description="Path to AprilTag map YAML (not used by follower right now).",
    )

    log_dir_arg = DeclareLaunchArgument(
        "log_dir",
        default_value=default_log_dir,
        description="Directory where HW5 pose logger will save trajectory JSON.",
    )

    camera_pkg = FindPackageShare('robot_vision_camera').find('robot_vision_camera')
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(camera_pkg, 'launch', 'robot_vision_camera.launch.py')
        )
    )

    apriltag_pkg = FindPackageShare('apriltag_ros').find('apriltag_ros')
    apriltag_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(apriltag_pkg, 'launch', 'apriltag_launch.py')
        )
    )

    camera_tf = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='hw_2_solution',
                executable='camera_tf',
                name='camera_tf',
                output='screen',
                emulate_tty=True,
            )
        ],
    )

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
        ],
    )

    velocity_mapping = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='hw5_coverage',
                executable='hw5_velocity_mapping',
                name='hw5_velocity_mapping',
                output='screen',
                emulate_tty=True,
            )
        ],
    )

    waypoint_follower = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='hw5_coverage',
                executable='hw5_waypoint_coverage',
                name='hw5_waypoint_coverage',  
                output='screen',
                emulate_tty=True,
                parameters=[
                    {"waypoint_file": LaunchConfiguration("waypoint_file")},
                    {"tag_yaml_file": LaunchConfiguration("tag_yaml_file")},
                ],
            )
        ],
    )

    return LaunchDescription([
        waypoint_file_arg,
        tag_yaml_arg,
        log_dir_arg,
        camera_launch,
        apriltag_launch,
        camera_tf,
        motor_controller,
        velocity_mapping,
        waypoint_follower,
    ])
