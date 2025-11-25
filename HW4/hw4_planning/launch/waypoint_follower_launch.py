# # waypoint_follower_launch.py
# """
# Launch file for following pre-generated waypoints.

# This is useful for testing a specific path without re-running the planner.
# Simply specify the JSON waypoint file and this will follow it.
# """

# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import IncludeLaunchDescription, TimerAction
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.substitutions import FindPackageShare
# from launch.substitutions import LaunchConfiguration
# from launch.actions import DeclareLaunchArgument
# import os


# def generate_launch_description():
#     """
#     Launch the waypoint follower system.
    
#     This starts:
#     1. Camera + AprilTag detection (for localization)
#     2. Motor control + velocity mapping
#     3. Waypoint follower (loads and follows JSON waypoints)
#     """
    
#     # Launch arguments
#     waypoints_file_arg = DeclareLaunchArgument(
#         'waypoints_file',
#         default_value='hw4_waypoints_safety.json',
#         description='Path to the JSON waypoint file'
#     )
    
#     # Find package paths
#     camera_pkg_path = FindPackageShare(
#         package='robot_vision_camera'
#     ).find('robot_vision_camera')
#     apriltag_pkg_path = FindPackageShare(
#         package='apriltag_ros'
#     ).find('apriltag_ros')
    
#     camera_launch_file = os.path.join(
#         camera_pkg_path, 'launch', 'robot_vision_camera.launch.py'
#     )
#     apriltag_launch_file = os.path.join(
#         apriltag_pkg_path, 'launch', 'apriltag_launch.py'
#     )
    
#     # Camera and AprilTag detection
#     camera_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(camera_launch_file)
#     )
    
#     apriltag_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(apriltag_launch_file)
#     )
    
#     # Motor control (with delay to let camera initialize)
#     motor_controller = TimerAction(
#         period=5.0,
#         actions=[
#             Node(
#                 package='hw_2_solution',
#                 executable='motor_control',
#                 name='motor_control',
#                 output='screen',
#                 emulate_tty=True,
#             )
#         ]
#     )
    
#     # Velocity mapping
#     velocity_mapping = TimerAction(
#         period=5.0,
#         actions=[
#             Node(
#                 package='hw4_planning',
#                 executable='hw4_velocity_mapping',
#                 name='hw4_velocity_mapping',
#                 output='screen',
#                 emulate_tty=True,
#             )
#         ]
#     )
    
#     # Camera TF
#     camera_tf = Node(
#         package='hw_2_solution',
#         executable='camera_tf',
#         name='camera_tf',
#         output='screen',
#         emulate_tty=True,
#     )
    
#     # Waypoint follower (with delay to let everything initialize)
#     waypoint_follower = TimerAction(
#         period=10.0,
#         actions=[
#             Node(
#                 package='hw4_planning',
#                 executable='waypoint_follower_node',
#                 name='waypoint_follower',
#                 output='screen',
#                 emulate_tty=True,
#                 parameters=[
#                     {
#                         'waypoints_file': LaunchConfiguration('waypoints_file'),
#                         'position_tolerance': 0.05,
#                         'angle_tolerance': 0.15,
#                         'max_linear_velocity': 0.2,
#                         'max_angular_velocity': 0.8,
#                     }
#                 ],
#             )
#         ]
#     )
    
#     return LaunchDescription([
#         waypoints_file_arg,
#         camera_launch,
#         apriltag_launch,
#         motor_controller,
#         velocity_mapping,
#         camera_tf,
#         waypoint_follower,
#     ])

# hw4_planning/launch/waypoint_follower_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    """
    Launch file for waypoint follower.

    Starts nodes in sequence:
    1. motor_control - hardware interface for motor commands
    2. hw4_velocity_mapping - converts /cmd_vel to motor commands
    3. [2s delay]
    4. waypoint_follower - follows pre-generated waypoint JSON file
    
    Usage:
        ros2 launch hw4_planning waypoint_follower_launch.py
        ros2 launch hw4_planning waypoint_follower_launch.py waypoint_file:=/path/to/waypoints.json
    """

    # Declare launch argument for waypoint file
    waypoint_file_arg = DeclareLaunchArgument(
        'waypoint_file',
        default_value='hw4_waypoints_safety.json',
        description='Path to the waypoint JSON file'
    )

    # Motor controller node
    motor_controller = Node(
        package='hw_2_solution',
        executable='motor_control',
        name='motor_control',
        output='screen',
        emulate_tty=True,
    )

    # Velocity mapping node (converts /cmd_vel to motor commands)
    velocity_mapping = Node(
        package='hw4_planning',
        executable='hw4_velocity_mapping',
        name='hw4_velocity_mapping',
        output='screen',
        emulate_tty=True,
    )

    # Waypoint follower node (with delay to let other nodes initialize)
    waypoint_follower = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='hw4_planning',
                executable='waypoint_follower',
                name='waypoint_follower_node',
                output='screen',
                emulate_tty=True,
                parameters=[
                    {'waypoint_file': LaunchConfiguration('waypoint_file')}
                ],
            )
        ]
    )

    return LaunchDescription([
        waypoint_file_arg,
        motor_controller,
        velocity_mapping,
        waypoint_follower,
    ])
