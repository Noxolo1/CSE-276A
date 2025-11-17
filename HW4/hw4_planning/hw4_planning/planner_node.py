# hw4_planning/hw4_planning/planner_node.py
#!/usr/bin/env python3
import math
import heapq
from typing import List, Tuple, Optional

import numpy as np
import rclpy

from rclpy.node import Node

# Reuse the HW2 solution node (localization + waypoint follower + PID)
from hw_2_solution.hw2_solution import Hw2SolutionNode  # type: ignore

# HW4 world configuration
from . import config as cfg


GridIndex = Tuple[int, int]
WorldPoint = Tuple[float, float]


class Hw4PlanningNode(Hw2SolutionNode):
    """
    HW4 planning node.

    This node reuses the entire HW2 solution node (Hw2SolutionNode) and only
    overrides how the waypoint list is constructed.

    - Localization: still using AprilTags and the tag map from hw_2_solution.
    - Control / PID / stages (rotate / drive / rotate): all from HW2.
    - Path planning: grid-based (A*), using an approximate cell decomposition
      of the 8x8 ft workspace with a central obstacle.

    You can switch between two planners via ROS 2 parameter 'planner_mode':
      - 'safety' : inflated obstacles => maximizes clearance
      - 'fast'   : minimal inflation => approximates shortest path
    """

    def __init__(self):
        super().__init__()  # sets up timers, PID, tag loading, etc.

        self.get_logger().info("HW4PlanningNode: initialized (extending HW2 solution).")

        # Store world / obstacle config from config.py
        self.world_min_x = cfg.WORLD_MIN_X
        self.world_max_x = cfg.WORLD_MAX_X
        self.world_min_y = cfg.WORLD_MIN_Y
        self.world_max_y = cfg.WORLD_MAX_Y

        self.obstacle_center_x = cfg.OBSTACLE_CENTER_X
        self.obstacle_center_y = cfg.OBSTACLE_CENTER_Y
        self.obstacle_half_x = cfg.OBSTACLE_HALF_M
        self.obstacle_half_y = cfg.OBSTACLE_HALF_M

        self.grid_resolution = cfg.GRID_RESOLUTION

        self.default_start = cfg.DEFAULT_START  # (x, y, yaw)
        self.default_goal = cfg.DEFAULT_GOAL    # (x, y, yaw)

        self.safety_inflation_radius = cfg.SAFETY_INFLATION_RADIUS_CELLS
        self.fast_inflation_radius = cfg.FAST_INFLATION_RADIUS_CELLS

        # Planner mode: "safety" or "fast"
        self.declare_parameter('planner_mode', 'safety')
        self.planner_mode = (
            self.get_parameter('planner_mode').get_parameter_value().string_value
        )
        if self.planner_mode not in ('safety', 'fast'):
            self.get_logger().warn(
                f"Invalid planner_mode '{self.planner_mode}', defaulting to 'safety'"
            )
            self.planner_mode = 'safety'

        self.get_logger().info(f"Planner mode: {self.planner_mode}")

        # Plan a path once at startup using DEFAULT_START / DEFAULT_GOAL.
        # The HW2 control loop will then follow these waypoints.
        self.plan_and_set_waypoints()

    # --------------------------------------------------------------------------
    # World / grid utilities
    # --------------------------------------------------------------------------

    def build_occupancy_grid(self) -> np.ndarray:
        """
        Create a binary occupancy grid for the workspace.

        Grid is indexed as [iy, ix], with:
          - 0 = free
          - 1 = occupied
        """
        width_m = self.world_max_x - self.world_min_x
        height_m = self.world_max_y - self.world_min_y

        nx = int(round(width_m / self.grid_resolution)) + 1
        ny = int(round(height_m / self.grid_resolution)) + 1

        grid = np.zeros((ny, nx), dtype=np.uint8)

        # Mark central rectangular obstacle
        for iy in range(ny):
            y = self.world_min_y + iy * self.grid_resolution
            for ix in range(nx):
                x = self.world_min_x + ix * self.grid_resolution

                in_obstacle_x = abs(x - self.obstacle_center_x) <= self.obstacle_half_x
                in_obstacle_y = abs(y - self.obstacle_center_y) <= self.obstacle_half_y

                if in_obstacle_x and in_obstacle_y:
                    grid[iy, ix] = 1

        return grid

    def inflate_obstacles(self, grid: np.ndarray, radius: int) -> np.ndarray:
        """Inflate obstacles by a Chebyshev radius (in grid cells)."""
        if radius <= 0:
            return grid.copy()

        ny, nx = grid.shape
        inflated = grid.copy()

        obstacle_indices = np.argwhere(grid == 1)
        for iy, ix in obstacle_indices:
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    jy = iy + dy
                    jx = ix + dx
                    if 0 <= jy < ny and 0 <= jx < nx:
                        inflated[jy, jx] = 1

        return inflated

    def world_to_grid(self, x: float, y: float) -> GridIndex:
        """Convert world (x,y) in meters to integer grid indices (ix, iy)."""
        ix = int(round((x - self.world_min_x) / self.grid_resolution))
        iy = int(round((y - self.world_min_y) / self.grid_resolution))
        return ix, iy

    def grid_to_world(self, ix: int, iy: int) -> WorldPoint:
        """Convert integer grid indices (ix, iy) back to world coordinates."""
        x = self.world_min_x + ix * self.grid_resolution
        y = self.world_min_y + iy * self.grid_resolution
        return x, y

    # --------------------------------------------------------------------------
    # A* planner on the grid
    # --------------------------------------------------------------------------

    def astar(
        self,
        occ_grid: np.ndarray,
        start_idx: GridIndex,
        goal_idx: GridIndex,
    ) -> Optional[List[GridIndex]]:
        """
        A* search on a 2D occupancy grid.

        occ_grid: 0 = free, 1 = occupied
        start_idx, goal_idx: (ix, iy)
        """
        ny, nx = occ_grid.shape
        sx, sy = start_idx
        gx, gy = goal_idx

        def in_bounds(ix: int, iy: int) -> bool:
            return 0 <= ix < nx and 0 <= iy < ny

        def heuristic(ix: int, iy: int) -> float:
            return math.hypot(gx - ix, gy - iy)

        # 8-connected grid (N, S, E, W, diagonals)
        neighbors = [
            (-1, 0), (1, 0),
            (0, -1), (0, 1),
            (-1, -1), (-1, 1),
            (1, -1), (1, 1),
        ]

        # (f_score, g_score, (ix, iy))
        open_heap: List[Tuple[float, float, GridIndex]] = []
        heapq.heappush(open_heap, (heuristic(sx, sy), 0.0, (sx, sy)))

        came_from: dict[GridIndex, GridIndex] = {}
        g_score: dict[GridIndex, float] = {(sx, sy): 0.0}
        closed: set[GridIndex] = set()

        while open_heap:
            f, g, current = heapq.heappop(open_heap)
            if current in closed:
                continue

            if current == (gx, gy):
                # Reconstruct path
                path: List[GridIndex] = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path

            closed.add(current)
            cx, cy = current

            for dx, dy in neighbors:
                nx_i = cx + dx
                ny_i = cy + dy

                if not in_bounds(nx_i, ny_i):
                    continue
                if occ_grid[ny_i, nx_i] == 1:
                    continue

                step_cost = math.hypot(dx, dy)
                tentative_g = g + step_cost

                neighbor = (nx_i, ny_i)
                if tentative_g < g_score.get(neighbor, float('inf')):
                    g_score[neighbor] = tentative_g
                    came_from[neighbor] = current
                    f_new = tentative_g + heuristic(nx_i, ny_i)
                    heapq.heappush(open_heap, (f_new, tentative_g, neighbor))

        return None  # No path found

    def simplify_grid_path(self, path: List[GridIndex]) -> List[GridIndex]:
        """
        Remove collinear intermediate points to reduce the number of waypoints.
        """
        if len(path) <= 2:
            return path

        simplified: List[GridIndex] = [path[0]]
        for i in range(1, len(path) - 1):
            x0, y0 = path[i - 1]
            x1, y1 = path[i]
            x2, y2 = path[i + 1]

            dx1 = x1 - x0
            dy1 = y1 - y0
            dx2 = x2 - x1
            dy2 = y2 - y1

            # If directions are collinear, skip the middle point
            if dx1 * dy2 - dy1 * dx2 == 0:
                continue

            simplified.append(path[i])

        simplified.append(path[-1])
        return simplified

    # --------------------------------------------------------------------------
    # High-level planning
    # --------------------------------------------------------------------------

    def plan_and_set_waypoints(self) -> None:
        """
        Build occupancy grid, run planner, convert to world-space waypoints,
        and override the HW2 waypoints.
        """
        # 1. Build base occupancy grid with central obstacle.
        base_grid = self.build_occupancy_grid()

        # 2. Inflate obstacles depending on planner mode.
        if self.planner_mode == 'safety':
            radius = self.safety_inflation_radius
        else:  # 'fast'
            radius = self.fast_inflation_radius

        occ_grid = self.inflate_obstacles(base_grid, radius)

        # 3. Map default start/goal (from config.py) into grid indices.
        start_x, start_y, start_yaw = self.default_start
        goal_x, goal_y, goal_yaw = self.default_goal

        sx, sy = self.world_to_grid(start_x, start_y)
        gx, gy = self.world_to_grid(goal_x, goal_y)

        ny, nx = occ_grid.shape
        if not (0 <= sx < nx and 0 <= sy < ny):
            self.get_logger().error("Start cell is out of bounds in the grid!")
            return
        if not (0 <= gx < nx and 0 <= gy < ny):
            self.get_logger().error("Goal cell is out of bounds in the grid!")
            return
        if occ_grid[sy, sx] == 1:
            self.get_logger().error("Start cell is inside an obstacle!")
            return
        if occ_grid[gy, gx] == 1:
            self.get_logger().error("Goal cell is inside an obstacle!")
            return

        # 4. Run A* on the inflated grid.
        grid_path = self.astar(occ_grid, (sx, sy), (gx, gy))
        if grid_path is None:
            self.get_logger().error("A* failed to find a path.")
            return

        grid_path = self.simplify_grid_path(grid_path)
        self.get_logger().info(f"Grid path has {len(grid_path)} points.")

        # 5. Convert grid path to world waypoints (x, y, yaw).
        waypoints_list: List[List[float]] = []
        for i, (ix, iy) in enumerate(grid_path):
            x, y = self.grid_to_world(ix, iy)

            if i < len(grid_path) - 1:
                nx_i, ny_i = grid_path[i + 1]
                nx_w, ny_w = self.grid_to_world(nx_i, ny_i)
                yaw = math.atan2(ny_w - y, nx_w - x)
            else:
                # Final orientation: use config default goal yaw
                yaw = goal_yaw

            waypoints_list.append([x, y, yaw])

        # --- THIS is the only place we touch the HW2 solution internals ---
        self.waypoints = np.array(waypoints_list, dtype=float)
        self.current_waypoint_idx = 0
        self.waypoint_reached = False
        self.stage = 'rotate_to_goal'

        self.get_logger().info(
            f"Planned {len(self.waypoints)} waypoints using mode '{self.planner_mode}'."
        )
        self.get_logger().info(
            f"Start: ({start_x:.3f}, {start_y:.3f}), "
            f"Goal: ({goal_x:.3f}, {goal_y:.3f})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = Hw4PlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('HW4 planning node stopped by keyboard interrupt')
    finally:
        node.stop_robot()  # inherited from Hw2SolutionNode
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
