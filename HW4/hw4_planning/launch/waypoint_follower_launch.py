from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    """
    starts nodes in sequence:
    1. motor_control 
    2. hw4_velocity_mapping 
    3. 5s delay
    4. waypoint_follower
    """

    
    waypoint_file_arg = DeclareLaunchArgument(
        'waypoint_file',
        default_value='hw4_waypoints_safety.json',
        description='Path to the waypoint JSON file'
    )

    motor_controller = Node(
        package='hw_2_solution',
        executable='motor_control',
        name='motor_control',
        output='screen',
        emulate_tty=True,
    )

    velocity_mapping = Node(
        package='hw4_planning',
        executable='hw4_velocity_mapping',
        name='hw4_velocity_mapping',
        output='screen',
        emulate_tty=True,
    )

    waypoint_follower = TimerAction(
        period=5.0,
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
