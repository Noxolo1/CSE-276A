"""
HW4 Main Planning Node

Orchestrates the A* path planner and waypoint follower.
This is the entry point for the HW4 homework assignment.
"""

import math
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node

from .astar_planner import AStarPlanner, Path
from .waypoint_follower_hw4 import WaypointFollowerHW4
from . import hw4_config as cfg


Waypoint = Tuple[float, float, float]  # (x, y, yaw)


class Hw4PlanningNode(Node):
    """
    Main HW4 planning node.

    Responsibilities:
    1. Initialize A* planner with world/obstacle configuration
    2. Generate path from start to goal
    3. Create waypoints with heading information
    4. Launch waypoint follower node to execute the plan
    5. Monitor execution and handle replanning if needed

    Each homework folder runs independently, so this node handles
    all HW4 functionality without depending on external homework packages.
    """

    def __init__(self):
        super().__init__("hw4_planning_node")

        self.get_logger().info("===== HW4 PLANNING NODE INITIALIZING =====")

        # Initialize A* planner
        self.planner = AStarPlanner(
            world_min_x=cfg.WORLD_MIN_X,
            world_max_x=cfg.WORLD_MAX_X,
            world_min_y=cfg.WORLD_MIN_Y,
            world_max_y=cfg.WORLD_MAX_Y,
            grid_resolution=cfg.GRID_RESOLUTION,
        )

        # Planner mode (safety vs fast)
        self.declare_parameter("planner_mode", "safety")
        self.planner_mode = (
            self.get_parameter("planner_mode").get_parameter_value().string_value
        )
        if self.planner_mode not in ("safety", "fast"):
            self.get_logger().warn(
                f"Invalid planner_mode '{self.planner_mode}', defaulting to 'safety'"
            )
            self.planner_mode = "safety"

        self.get_logger().info(f"Planner mode: {self.planner_mode}")

        # Inflation radius based on mode
        inflation_radius = (
            cfg.SAFETY_INFLATION_RADIUS_CELLS
            if self.planner_mode == "safety"
            else cfg.FAST_INFLATION_RADIUS_CELLS
        )

        self.get_logger().info(
            f"Obstacle inflation: {inflation_radius} cells "
            f"({inflation_radius * cfg.GRID_RESOLUTION:.2f} m)"
        )

        # Build occupancy grid
        self.grid = self.planner.build_occupancy_grid(
            obstacle_centers=cfg.OBSTACLES,
            inflation_radius_cells=inflation_radius,
        )

        self.get_logger().info(
            f"Grid size: {self.grid.shape[1]} x {self.grid.shape[0]} cells"
        )

        # Plan a path from start to goal
        start = cfg.DEFAULT_START[:2]  # (x, y)
        goal = cfg.DEFAULT_GOAL[:2]  # (x, y)

        self.get_logger().info(f"Planning from {start} to {goal}")

        path = self.planner.plan(start, goal, self.grid)

        if path is None:
            self.get_logger().error("No path found!")
            raise RuntimeError("A* planning failed - no path exists")

        self.get_logger().info(f"Path found with {len(path)} waypoints")

        # Convert path to waypoints with heading information
        waypoints = self._path_to_waypoints(path)

        self.get_logger().info("Waypoints with heading:")
        for i, wp in enumerate(waypoints):
            self.get_logger().info(
                f"  {i}: ({wp[0]:.3f}, {wp[1]:.3f}), yaw={math.degrees(wp[2]):.1f}°"
            )

        # Create waypoint follower with generated path
        self.follower = WaypointFollowerHW4(waypoints=waypoints)

        self.get_logger().info("===== HW4 PLANNING NODE READY =====")

    def _path_to_waypoints(self, path: Path) -> List[Waypoint]:
        """
        Convert smooth path to waypoints with heading information.

        Strategy:
        1. Use path points directly as waypoints
        2. Compute heading at each waypoint based on next point direction
        3. Final waypoint heading = desired final heading from config

        Args:
            path: List of (x, y) positions from A*

        Returns:
            List of (x, y, yaw) waypoints
        """
        waypoints = []

        for i, (x, y) in enumerate(path):
            if i < len(path) - 1:
                # Heading toward next waypoint
                nx, ny = path[i + 1]
                yaw = math.atan2(ny - y, nx - x)
            else:
                # Final waypoint: use goal heading from config
                yaw = cfg.DEFAULT_GOAL[2]

            waypoints.append((x, y, yaw))

        # Optionally: downsample waypoints to reduce number
        # (e.g., keep every 5th point for faster execution)
        # This is useful if A* creates many intermediate points

        return waypoints

    def log_grid(self, filename: str = "occupancy_grid.txt"):
        """
        Save occupancy grid to file for debugging/visualization.

        Args:
            filename: Output filename in home directory
        """
        import os

        path = os.path.expanduser(f"~/{filename}")
        with open(path, "w") as f:
            f.write(f"# Grid size: {self.grid.shape[1]} x {self.grid.shape[0]}\n")
            f.write(f"# Resolution: {cfg.GRID_RESOLUTION} m/cell\n")
            f.write(f"# World bounds: ({cfg.WORLD_MIN_X}, {cfg.WORLD_MIN_Y}) to "
                    f"({cfg.WORLD_MAX_X}, {cfg.WORLD_MAX_Y})\n")
            f.write(f"# Planner mode: {self.planner_mode}\n")
            f.write("# 0 = free, 1 = occupied\n\n")

            for row in self.grid:
                f.write("".join(str(int(cell)) for cell in row) + "\n")

        self.get_logger().info(f"Grid saved to {path}")


def main():
    rclpy.init()
    node = Hw4PlanningNode()

    # Log the grid for debugging
    node.log_grid()

    # Spin the node (runs waypoint follower)
    rclpy.spin(node.follower)  # Spin the follower instead

    rclpy.shutdown()


if __name__ == "__main__":
    main()
