from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Camera + AprilTags (same as HW2/HW3/HW4; adjust to your actual file names)
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

    # Static TF base_link -> camera_frame (HW2 reference node)
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
                executable='hw5_velocity_mapping',  # from earlier
                name='hw5_velocity_mapping',
                output='screen',
                emulate_tty=True,
            )
        ]
    )

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
                    {'trajectory_log_file': 'hw5_coverage_trajectory.json'},
                    # you can also override k_v, k_w, explore_speed, etc. here
                ],
            )
        ]
    )

    return LaunchDescription([
        camera_launch,
        apriltag_launch,
        camera_tf,
        motor_controller,
        velocity_mapping,
        coverage_node,
    ])
