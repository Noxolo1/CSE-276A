from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration

import os


def generate_launch_description():
    # ------------------------------------------------------------------
    # Paths for waypoint JSON and AprilTag YAML
    # ------------------------------------------------------------------
    default_wp_path = os.path.join(
        os.path.expanduser("~"),
        "ros2_ws", "rubikpi_ros2", "hw5_coverage", "hw5_coverage",
        "hw5_waypoints_lawnmower.json",
    )

    default_tag_yaml_path = os.path.join(
        os.path.expanduser("~"),
        "ros2_ws", "rubikpi_ros2", "hw5_coverage", "hw5_coverage",
        "apriltags_position.yaml",
    )

    # log_dir: put EKF trajectory logs next to waypoint file
    default_log_dir = os.path.dirname(default_wp_path)

    waypoint_file_arg = DeclareLaunchArgument(
        "waypoint_file",
        default_value=default_wp_path,
        description="Path to HW5 lawnmower waypoint JSON file."
    )

    tag_yaml_arg = DeclareLaunchArgument(
        "tag_yaml_file",
        default_value=default_tag_yaml_path,
        description="Path to AprilTag map YAML (not used by follower right now)."
    )

    log_dir_arg = DeclareLaunchArgument(
        "log_dir",
        default_value=default_log_dir,
        description="Directory where HW5 pose logger will save trajectory JSON."
    )

    # ------------------------------------------------------------------
    # Camera + AprilTags (same as your previous homeworks)
    # ------------------------------------------------------------------
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

    # ------------------------------------------------------------------
    # Static TF base_link -> camera_frame (from HW2)
    # ------------------------------------------------------------------
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
        ]
    )

    # ------------------------------------------------------------------
    # Motor control + HW5 velocity mapping
    # ------------------------------------------------------------------
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
        ]
    )

    # ------------------------------------------------------------------
    # HW4-style waypoint follower (dead-reckoning internal state)
    # ------------------------------------------------------------------
    waypoint_follower = TimerAction(
        period=10.0,   # start after motors / tf are up
        actions=[
            Node(
                package='hw5_coverage',
                executable='hw5_waypoint_follower',
                name='hw5_waypoint_follower',
                output='screen',
                emulate_tty=True,
                parameters=[
                    {"waypoint_file": LaunchConfiguration("waypoint_file")},
                    {"tag_yaml_file": LaunchConfiguration("tag_yaml_file")},
                ],
            )
        ]
    )

        # ------------------------------------------------------------------
    # HW3 EKF-SLAM node (publishes /slam_pose), copied from hw3_slam.launch.py
    # ------------------------------------------------------------------
    ekf_slam_node = TimerAction(
        period=10.0,   # same delay you used in HW3
        actions=[
            Node(
                package='hw3_slam',
                executable='hw3_slam',      # <- THIS is the correct executable
                name='hw3_slam_node',
                output='screen',
                emulate_tty=True,
                parameters=[{
                    'dt': 0.05,
                    # you can reuse HW3 logging or change it for HW5:
                    'log_data': True,
                    'log_dir': '/tmp/hw3_slam_logs',
                }]
            )
        ]
    )

    # ------------------------------------------------------------------
    # Pose logger: subscribes to /slam_pose and logs JSON trajectory
    # ------------------------------------------------------------------
    pose_logger = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='hw5_coverage',
                executable='hw5_pose_logger',
                name='hw5_pose_logger',
                output='screen',
                emulate_tty=True,
                parameters=[
                    {"pose_topic": "/slam_pose"},
                    {"log_dir": LaunchConfiguration("log_dir")},
                ],
            )
        ]
    )

    # ------------------------------------------------------------------
    # AprilTag-based pose logger (does NOT influence control)
    # ------------------------------------------------------------------
    tag_pose_logger = TimerAction(
        period=10.0,  # start after camera/apriltags are up
        actions=[
            Node(
                package='hw5_coverage',
                executable='hw5_tag_pose_logger',
                name='hw5_tag_pose_logger',
                output='screen',
                emulate_tty=True,
                parameters=[
                    {"tag_yaml_file": LaunchConfiguration("tag_yaml_file")},
                    {"log_dir": LaunchConfiguration("log_dir")},
                    {"base_frame": "base_link"},
                    {"log_rate": 5.0},  # Hz
                ],
            )
        ]
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
        ekf_slam_node,      # <- include this
        waypoint_follower,
        pose_logger,        # <- and this
        log_dir_arg,
        tag_pose_logger,
    ])
