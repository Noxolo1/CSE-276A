# hw4_planning/hw4_planning/json_waypoint_follower.py
#!/usr/bin/env python3

import os
import json
from typing import List, Dict, Any

import numpy as np
import rclpy
from rclpy.node import Node

from hw_2_solution.hw2_solution import Hw2SolutionNode  # reuse HW2 controller/localization


class JsonWaypointFollowerNode(Hw2SolutionNode):
    """
    Very simple waypoint follower that reads a JSON file of waypoints and
    reuses the HW2 solution code to follow them.

    Expected JSON formats (either is accepted):

    1) List of dicts:
       [
         {"index": 0, "x": 0.0, "y": 0.0, "yaw": 0.0},
         {"index": 1, "x": 0.1, "y": 0.0, "yaw": 0.0},
         ...
       ]

    2) List of lists:
       [
         [x0, y0, yaw0],
         [x1, y1, yaw1],
         ...
       ]
    """

    def __init__(self) -> None:
        super().__init__()  # set up HW2 timers, publishers, etc.

        # Parameters:
        # - waypoints_json_path: full path to JSON file (optional)
        # - planner_mode: "safety" / "fast" (used only to pick default filename)
        self.declare_parameter("waypoints_json_path", "")
        self.declare_parameter("planner_mode", "safety")

        explicit_path = (
            self.get_parameter("waypoints_json_path")
            .get_parameter_value()
            .string_value
        )
        mode = (
            self.get_parameter("planner_mode")
            .get_parameter_value()
            .string_value
        )

        # If user did not pass a path, fall back to the default we used
        # when saving the JSON: ~/ros2_ws/rubikpi_ros2/hw4_waypoints_<mode>.json
        if explicit_path:
            json_path = explicit_path
        else:
            workspace_root = os.path.expanduser("~/ros2_ws/rubikpi_ros2")
            json_path = os.path.join(
                workspace_root,
                f"hw4_waypoints_{mode}.json"
            )

        self.get_logger().info(f"Loading waypoints from JSON: {json_path}")

        try:
            with open(json_path, "r") as f:
                data = json.load(f)
        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints JSON: {e!r}")
            # stop motors for safety and bail out
            self.stop_robot()
            raise

        waypoints_list = self._parse_waypoints(data)
        if not waypoints_list:
            self.get_logger().error("No valid waypoints loaded from JSON; aborting.")
            self.stop_robot()
            raise RuntimeError("No valid waypoints in JSON")

        # Convert to numpy array of shape (N, 3): [x, y, yaw]
        self.waypoints = np.array(waypoints_list, dtype=float)
        self.current_waypoint_idx = 0
        self.waypoint_reached = False
        self.stage = "rotate_to_goal"  # same initial stage as HW2

        self.get_logger().info(
            f"Loaded {len(self.waypoints)} waypoints from JSON."
        )
        self.get_logger().info(
            "Start waypoint: x={:.3f} m, y={:.3f} m, yaw={:.3f} rad".format(
                self.waypoints[0, 0],
                self.waypoints[0, 1],
                self.waypoints[0, 2],
            )
        )
        self.get_logger().info(
            "Goal waypoint:  x={:.3f} m, y={:.3f} m, yaw={:.3f} rad".format(
                self.waypoints[-1, 0],
                self.waypoints[-1, 1],
                self.waypoints[-1, 2],
            )
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _parse_waypoints(self, data: Any) -> List[List[float]]:
        """
        Parse various JSON formats into a list of [x, y, yaw] floats.
        """
        waypoints: List[List[float]] = []

        if not isinstance(data, list) or not data:
            self.get_logger().error("Waypoint JSON must be a non-empty list.")
            return waypoints

        first = data[0]

        # Case 1: list of dicts
        if isinstance(first, dict):
            for item in data:
                try:
                    x = float(item["x"])
                    y = float(item["y"])
                    yaw = float(item.get("yaw", 0.0))
                    waypoints.append([x, y, yaw])
                except Exception as e:
                    self.get_logger().warn(
                        f"Skipping invalid waypoint dict {item!r}: {e!r}"
                    )

        # Case 2: list of lists/tuples
        elif isinstance(first, (list, tuple)):
            for item in data:
                if not isinstance(item, (list, tuple)):
                    self.get_logger().warn(
                        f"Skipping non-list waypoint item: {item!r}"
                    )
                    continue

                if len(item) >= 3:
                    x, y, yaw = item[0], item[1], item[2]
                elif len(item) == 2:
                    x, y = item[0], item[1]
                    yaw = 0.0
                else:
                    self.get_logger().warn(
                        f"Skipping too-short waypoint: {item!r}"
                    )
                    continue

                try:
                    waypoints.append([float(x), float(y), float(yaw)])
                except Exception as e:
                    self.get_logger().warn(
                        f"Skipping invalid waypoint list {item!r}: {e!r}"
                    )

        else:
            self.get_logger().error(
                "Unknown waypoint JSON format; expected list of dicts or list of lists."
            )

        return waypoints


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Node = JsonWaypointFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("JSON waypoint follower stopped by user.")
    finally:
        node.stop_robot()  # from Hw2SolutionNode
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
