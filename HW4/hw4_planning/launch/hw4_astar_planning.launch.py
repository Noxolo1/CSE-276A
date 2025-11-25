"""
HW4 Planning Launch File

Launches the complete HW4 planning and execution pipeline:
1. Pose estimation node (localization)
2. A* planner + waypoint follower (planning)
3. Motor controller (actuation)

This launch file is self-contained and runs independently from other assignments.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Generate launch description for HW4 planning node.

    Configurable parameters:
    - planner_mode: 'safety' (default) or 'fast'
    """

    # Declare launch arguments
    planner_mode_arg = DeclareLaunchArgument(
        "planner_mode",
        default_value="safety",
        description="Planner mode: 'safety' (inflated) or 'fast' (minimal inflation)",
    )

    # Pose estimation node (localization using AprilTags)
    localization_node = Node(
        package="hw4_planning",
        executable="localization_hw4_node",
        name="hw4_localization",
        output="screen",
    )

    # Main planning node (A* + waypoint follower)
    planning_node = Node(
        package="hw4_planning",
        executable="planning_node",
        name="hw4_planning",
        output="screen",
        parameters=[
            {"planner_mode": LaunchConfiguration("planner_mode")},
        ],
    )

    # Motor controller node (from HW2, but used here)
    motor_controller_node = Node(
        package="hw4_planning",
        executable="motor_controller_node",
        name="hw4_motor_controller",
        output="screen",
    )

    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(planner_mode_arg)

    # Add nodes
    ld.add_action(localization_node)
    ld.add_action(planning_node)
    ld.add_action(motor_controller_node)

    return ld
