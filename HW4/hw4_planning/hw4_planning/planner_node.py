# # hw4_planning/hw4_planning/planner_node.py
# #!/usr/bin/env python3
# import math
# import heapq
# from typing import List, Tuple, Optional

# import numpy as np
# import rclpy

# from rclpy.node import Node

# # Reuse the HW2 solution node (localization + waypoint follower + PID)
# from hw_2_solution.hw2_solution import Hw2SolutionNode  # type: ignore

# # HW4 world configuration
# from . import config as cfg


# GridIndex = Tuple[int, int]
# WorldPoint = Tuple[float, float]


# # class Hw4PlanningNode(Hw2SolutionNode):
# #     """
# #     HW4 planning node.

# #     This node reuses the entire HW2 solution node (Hw2SolutionNode) and only
# #     overrides how the waypoint list is constructed.

# #     - Localization: still using AprilTags and the tag map from hw_2_solution.
# #     - Control / PID / stages (rotate / drive / rotate): all from HW2.
# #     - Path planning: grid-based (A*), using an approximate cell decomposition
# #       of the 8x8 ft workspace with a central obstacle.

# #     You can switch between two planners via ROS 2 parameter 'planner_mode':
# #       - 'safety' : inflated obstacles => maximizes clearance
# #       - 'fast'   : minimal inflation => approximates shortest path
# #     """

# #     def __init__(self):
# #         super().__init__()  # sets up timers, PID, tag loading, etc.

# #         self.get_logger().info("HW4PlanningNode: initialized (extending HW2 solution).")

# #         # Store world / obstacle config from config.py
# #         self.world_min_x = cfg.WORLD_MIN_X
# #         self.world_max_x = cfg.WORLD_MAX_X
# #         self.world_min_y = cfg.WORLD_MIN_Y
# #         self.world_max_y = cfg.WORLD_MAX_Y

# #         self.obstacle_center_x = cfg.OBSTACLE_CENTER_X
# #         self.obstacle_center_y = cfg.OBSTACLE_CENTER_Y
# #         self.obstacle_half_x = cfg.OBSTACLE_HALF_M
# #         self.obstacle_half_y = cfg.OBSTACLE_HALF_M

# #         self.grid_resolution = cfg.GRID_RESOLUTION

# #         self.default_start = cfg.DEFAULT_START  # (x, y, yaw)
# #         self.default_goal = cfg.DEFAULT_GOAL    # (x, y, yaw)

# #         self.safety_inflation_radius = cfg.SAFETY_INFLATION_RADIUS_CELLS
# #         self.fast_inflation_radius = cfg.FAST_INFLATION_RADIUS_CELLS

# #         # Planner mode: "safety" or "fast"
# #         self.declare_parameter('planner_mode', 'safety')
# #         self.planner_mode = (
# #             self.get_parameter('planner_mode').get_parameter_value().string_value
# #         )
# #         if self.planner_mode not in ('safety', 'fast'):
# #             self.get_logger().warn(
# #                 f"Invalid planner_mode '{self.planner_mode}', defaulting to 'safety'"
# #             )
# #             self.planner_mode = 'safety'

# #         self.get_logger().info(f"Planner mode: {self.planner_mode}")

# #         # Plan a path once at startup using DEFAULT_START / DEFAULT_GOAL.
# #         # The HW2 control loop will then follow these waypoints.
# #         self.plan_and_set_waypoints()

# #     # --------------------------------------------------------------------------
# #     # World / grid utilities
# #     # --------------------------------------------------------------------------

# #     def build_occupancy_grid(self) -> np.ndarray:
# #         """
# #         Create a binary occupancy grid for the workspace.

# #         Grid is indexed as [iy, ix], with:
# #           - 0 = free
# #           - 1 = occupied
# #         """
# #         width_m = self.world_max_x - self.world_min_x
# #         height_m = self.world_max_y - self.world_min_y

# #         nx = int(round(width_m / self.grid_resolution)) + 1
# #         ny = int(round(height_m / self.grid_resolution)) + 1

# #         grid = np.zeros((ny, nx), dtype=np.uint8)

# #         # Mark central rectangular obstacle
# #         for iy in range(ny):
# #             y = self.world_min_y + iy * self.grid_resolution
# #             for ix in range(nx):
# #                 x = self.world_min_x + ix * self.grid_resolution

# #                 in_obstacle_x = abs(x - self.obstacle_center_x) <= self.obstacle_half_x
# #                 in_obstacle_y = abs(y - self.obstacle_center_y) <= self.obstacle_half_y

# #                 if in_obstacle_x and in_obstacle_y:
# #                     grid[iy, ix] = 1

# #         return grid

# #     def inflate_obstacles(self, grid: np.ndarray, radius: int) -> np.ndarray:
# #         """Inflate obstacles by a Chebyshev radius (in grid cells)."""
# #         if radius <= 0:
# #             return grid.copy()

# #         ny, nx = grid.shape
# #         inflated = grid.copy()

# #         obstacle_indices = np.argwhere(grid == 1)
# #         for iy, ix in obstacle_indices:
# #             for dy in range(-radius, radius + 1):
# #                 for dx in range(-radius, radius + 1):
# #                     jy = iy + dy
# #                     jx = ix + dx
# #                     if 0 <= jy < ny and 0 <= jx < nx:
# #                         inflated[jy, jx] = 1

# #         return inflated

# #     def world_to_grid(self, x: float, y: float) -> GridIndex:
# #         """Convert world (x,y) in meters to integer grid indices (ix, iy)."""
# #         ix = int(round((x - self.world_min_x) / self.grid_resolution))
# #         iy = int(round((y - self.world_min_y) / self.grid_resolution))
# #         return ix, iy

# #     def grid_to_world(self, ix: int, iy: int) -> WorldPoint:
# #         """Convert integer grid indices (ix, iy) back to world coordinates."""
# #         x = self.world_min_x + ix * self.grid_resolution
# #         y = self.world_min_y + iy * self.grid_resolution
# #         return x, y

    

# #     # --------------------------------------------------------------------------
# #     # A* planner on the grid
# #     # --------------------------------------------------------------------------

# #     def astar(
# #         self,
# #         occ_grid: np.ndarray,
# #         start_idx: GridIndex,
# #         goal_idx: GridIndex,
# #     ) -> Optional[List[GridIndex]]:
# #         """
# #         A* search on a 2D occupancy grid.

# #         occ_grid: 0 = free, 1 = occupied
# #         start_idx, goal_idx: (ix, iy)
# #         """
# #         ny, nx = occ_grid.shape
# #         sx, sy = start_idx
# #         gx, gy = goal_idx

# #         def in_bounds(ix: int, iy: int) -> bool:
# #             return 0 <= ix < nx and 0 <= iy < ny

# #         def heuristic(ix: int, iy: int) -> float:
# #             return math.hypot(gx - ix, gy - iy)

# #         # 8-connected grid (N, S, E, W, diagonals)
# #         neighbors = [
# #             (-1, 0), (1, 0),
# #             (0, -1), (0, 1),
# #             (-1, -1), (-1, 1),
# #             (1, -1), (1, 1),
# #         ]

# #         # (f_score, g_score, (ix, iy))
# #         open_heap: List[Tuple[float, float, GridIndex]] = []
# #         heapq.heappush(open_heap, (heuristic(sx, sy), 0.0, (sx, sy)))

# #         came_from: dict[GridIndex, GridIndex] = {}
# #         g_score: dict[GridIndex, float] = {(sx, sy): 0.0}
# #         closed: set[GridIndex] = set()

# #         while open_heap:
# #             f, g, current = heapq.heappop(open_heap)
# #             if current in closed:
# #                 continue

# #             if current == (gx, gy):
# #                 # Reconstruct path
# #                 path: List[GridIndex] = [current]
# #                 while current in came_from:
# #                     current = came_from[current]
# #                     path.append(current)
# #                 path.reverse()
# #                 return path

# #             closed.add(current)
# #             cx, cy = current

# #             for dx, dy in neighbors:
# #                 nx_i = cx + dx
# #                 ny_i = cy + dy

# #                 if not in_bounds(nx_i, ny_i):
# #                     continue
# #                 if occ_grid[ny_i, nx_i] == 1:
# #                     continue

# #                 step_cost = math.hypot(dx, dy)
# #                 tentative_g = g + step_cost

# #                 neighbor = (nx_i, ny_i)
# #                 if tentative_g < g_score.get(neighbor, float('inf')):
# #                     g_score[neighbor] = tentative_g
# #                     came_from[neighbor] = current
# #                     f_new = tentative_g + heuristic(nx_i, ny_i)
# #                     heapq.heappush(open_heap, (f_new, tentative_g, neighbor))

# #         return None  # No path found

# #     def simplify_grid_path(self, path: List[GridIndex]) -> List[GridIndex]:
# #         """
# #         Remove collinear intermediate points to reduce the number of waypoints.
# #         """
# #         if len(path) <= 2:
# #             return path

# #         simplified: List[GridIndex] = [path[0]]
# #         for i in range(1, len(path) - 1):
# #             x0, y0 = path[i - 1]
# #             x1, y1 = path[i]
# #             x2, y2 = path[i + 1]

# #             dx1 = x1 - x0
# #             dy1 = y1 - y0
# #             dx2 = x2 - x1
# #             dy2 = y2 - y1

# #             # If directions are collinear, skip the middle point
# #             if dx1 * dy2 - dy1 * dx2 == 0:
# #                 continue

# #             simplified.append(path[i])

# #         simplified.append(path[-1])
# #         return simplified

# #     # --------------------------------------------------------------------------
# #     # High-level planning
# #     # --------------------------------------------------------------------------

# #     def plan_and_set_waypoints(self) -> None:
# #         """
# #         Build occupancy grid, run planner, convert to world-space waypoints,
# #         and override the HW2 waypoints.
# #         """
# #         # 1. Build base occupancy grid with central obstacle.
# #         base_grid = self.build_occupancy_grid()

# #         # 2. Inflate obstacles depending on planner mode.
# #         if self.planner_mode == 'safety':
# #             radius = self.safety_inflation_radius
# #         else:  # 'fast'
# #             radius = self.fast_inflation_radius

# #         occ_grid = self.inflate_obstacles(base_grid, radius)

# #         # 3. Map default start/goal (from config.py) into grid indices.
# #         start_x, start_y, start_yaw = self.default_start
# #         goal_x, goal_y, goal_yaw = self.default_goal

# #         sx, sy = self.world_to_grid(start_x, start_y)
# #         gx, gy = self.world_to_grid(goal_x, goal_y)

# #         ny, nx = occ_grid.shape
# #         if not (0 <= sx < nx and 0 <= sy < ny):
# #             self.get_logger().error("Start cell is out of bounds in the grid!")
# #             return
# #         if not (0 <= gx < nx and 0 <= gy < ny):
# #             self.get_logger().error("Goal cell is out of bounds in the grid!")
# #             return
# #         if occ_grid[sy, sx] == 1:
# #             self.get_logger().error("Start cell is inside an obstacle!")
# #             return
# #         if occ_grid[gy, gx] == 1:
# #             self.get_logger().error("Goal cell is inside an obstacle!")
# #             return

# #         # 4. Run A* on the inflated grid.
# #         grid_path = self.astar(occ_grid, (sx, sy), (gx, gy))
# #         if grid_path is None:
# #             self.get_logger().error("A* failed to find a path.")
# #             return

# #         grid_path = self.simplify_grid_path(grid_path)
# #         self.get_logger().info(f"Grid path has {len(grid_path)} points.")

# #         # 5. Convert grid path to world waypoints (x, y, yaw).
# #         waypoints_list: List[List[float]] = []
# #         for i, (ix, iy) in enumerate(grid_path):
# #             x, y = self.grid_to_world(ix, iy)

# #             if i < len(grid_path) - 1:
# #                 nx_i, ny_i = grid_path[i + 1]
# #                 nx_w, ny_w = self.grid_to_world(nx_i, ny_i)
# #                 yaw = math.atan2(ny_w - y, nx_w - x)
# #             else:
# #                 # Final orientation: use config default goal yaw
# #                 yaw = goal_yaw

# #             waypoints_list.append([x, y, yaw])

# #         # --- THIS is the only place we touch the HW2 solution internals ---
# #         self.waypoints = np.array(waypoints_list, dtype=float)
# #         self.current_waypoint_idx = 0
# #         self.waypoint_reached = False
# #         self.stage = 'rotate_to_goal'

# #         self.get_logger().info(
# #             f"Planned {len(self.waypoints)} waypoints using mode '{self.planner_mode}'."
# #         )
# #         self.get_logger().info(
# #             f"Start: ({start_x:.3f}, {start_y:.3f}), "
# #             f"Goal: ({goal_x:.3f}, {goal_y:.3f})"
# #         )

# # ---------------------------------------------------------------
# # Offline grid-based planner that uses the AprilTag map
# # ---------------------------------------------------------------

# class OfflineGridPlanner:
#     """Builds an occupancy grid from the AprilTag map and runs A* offline."""

#     def __init__(
#         self,
#         apriltag_yaml_path: str,
#         resolution: float = cfg.GRID_RESOLUTION,
#         margin: float = cfg.MAP_MARGIN_M,
#         obstacle_tag_ids: Tuple[int, int, int, int] = cfg.OBSTACLE_TAG_IDS,
#     ) -> None:
#         self.resolution = resolution
#         self.margin = margin
#         self.tags = self._load_tags(apriltag_yaml_path)
#         self.obstacle_tag_ids = obstacle_tag_ids

#         # Derived world bounds (include all tags + start + goal)
#         self.world_min_x, self.world_max_x, self.world_min_y, self.world_max_y = (
#             self._compute_world_bounds()
#         )
#         self.nx = int(math.ceil((self.world_max_x - self.world_min_x) / self.resolution))
#         self.ny = int(math.ceil((self.world_max_y - self.world_min_y) / self.resolution))

#         self.occ_grid = self._build_base_occupancy()

#     def _load_tags(self, path: str) -> dict[int, dict]:
#         with open(path, "r") as f:
#             data = yaml.safe_load(f)
#         # Map id -> tag dict
#         return {int(t["id"]): t for t in data["apriltags"]}

#     def _compute_world_bounds(self) -> Tuple[float, float, float, float]:
#         xs = [t["x"] for t in self.tags.values()]
#         ys = [t["y"] for t in self.tags.values()]
#         # Also include start/goal
#         xs += [cfg.DEFAULT_START[0], cfg.DEFAULT_GOAL[0]]
#         ys += [cfg.DEFAULT_START[1], cfg.DEFAULT_GOAL[1]]

#         min_x = min(xs) - self.margin
#         max_x = max(xs) + self.margin
#         min_y = min(ys) - self.margin
#         max_y = max(ys) + self.margin
#         return min_x, max_x, min_y, max_y

#     # --- world <-> grid helpers ---------------------------------------------

#     def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
#         ix = int((x - self.world_min_x) / self.resolution)
#         iy = int((y - self.world_min_y) / self.resolution)
#         return ix, iy

#     def grid_to_world(self, ix: int, iy: int) -> Tuple[float, float]:
#         x = self.world_min_x + (ix + 0.5) * self.resolution
#         y = self.world_min_y + (iy + 0.5) * self.resolution
#         return x, y

#     # --- occupancy grid construction ----------------------------------------

#     def _build_base_occupancy(self) -> np.ndarray:
#         """0 = free, 1 = obstacle (only the central obstacle)."""
#         occ = np.zeros((self.ny, self.nx), dtype=np.uint8)

#         # Get bounding box of obstacle tags (8-11 by config)
#         obs_tags = [self.tags[i] for i in self.obstacle_tag_ids]
#         oxs = [t["x"] for t in obs_tags]
#         oys = [t["y"] for t in obs_tags]
#         obs_min_x, obs_max_x = min(oxs), max(oxs)
#         obs_min_y, obs_max_y = min(oys), max(oys)

#         # Small padding around obstacle (so we don't skim the tags)
#         pad = 0.05  # 5 cm

#         for iy in range(self.ny):
#             cy = self.world_min_y + (iy + 0.5) * self.resolution
#             for ix in range(self.nx):
#                 cx = self.world_min_x + (ix + 0.5) * self.resolution
#                 if (
#                     obs_min_x - pad <= cx <= obs_max_x + pad
#                     and obs_min_y - pad <= cy <= obs_max_y + pad
#                 ):
#                     occ[iy, ix] = 1

#         return occ

#     # --- inflation and A* ---------------------------------------------------

#     @staticmethod
#     def inflate_occ_grid(occ_grid: np.ndarray, radius_cells: int) -> np.ndarray:
#         """Naive square-disk inflation: mark neighbors within radius as occupied."""
#         if radius_cells <= 0:
#             return occ_grid.copy()

#         ny, nx = occ_grid.shape
#         inflated = occ_grid.copy()
#         for iy in range(ny):
#             for ix in range(nx):
#                 if occ_grid[iy, ix]:
#                     x0 = max(0, ix - radius_cells)
#                     x1 = min(nx, ix + radius_cells + 1)
#                     y0 = max(0, iy - radius_cells)
#                     y1 = min(ny, iy + radius_cells + 1)
#                     inflated[y0:y1, x0:x1] = 1
#         return inflated

#     @staticmethod
#     def astar(
#         occ_grid: np.ndarray,
#         start_idx: Tuple[int, int],
#         goal_idx: Tuple[int, int],
#     ) -> Optional[List[Tuple[int, int]]]:
#         """A* search on 2D grid (8-connected)."""
#         ny, nx = occ_grid.shape
#         sx, sy = start_idx
#         gx, gy = goal_idx

#         def in_bounds(ix: int, iy: int) -> bool:
#             return 0 <= ix < nx and 0 <= iy < ny

#         def is_free(ix: int, iy: int) -> bool:
#             return occ_grid[iy, ix] == 0

#         moves = [
#             (-1, 0),
#             (1, 0),
#             (0, -1),
#             (0, 1),
#             (-1, -1),
#             (-1, 1),
#             (1, -1),
#             (1, 1),
#         ]

#         def heuristic(ix: int, iy: int) -> float:
#             return math.hypot(gx - ix, gy - iy)

#         open_set: List[Tuple[float, float, Tuple[int, int]]] = []
#         heapq.heappush(open_set, (heuristic(sx, sy), 0.0, (sx, sy)))
#         came_from: dict[Tuple[int, int], Tuple[int, int]] = {}
#         g_score: dict[Tuple[int, int], float] = {(sx, sy): 0.0}

#         while open_set:
#             f, g, (ix, iy) = heapq.heappop(open_set)
#             if (ix, iy) == (gx, gy):
#                 # Reconstruct best path
#                 path: List[Tuple[int, int]] = [(ix, iy)]
#                 while (ix, iy) in came_from:
#                     ix, iy = came_from[(ix, iy)]
#                     path.append((ix, iy))
#                 path.reverse()
#                 return path

#             for dx, dy in moves:
#                 nix, niy = ix + dx, iy + dy
#                 if not in_bounds(nix, niy) or not is_free(nix, niy):
#                     continue

#                 step_cost = math.hypot(dx, dy)
#                 tentative_g = g + step_cost
#                 if tentative_g < g_score.get((nix, niy), float("inf")):
#                     g_score[(nix, niy)] = tentative_g
#                     came_from[(nix, niy)] = (ix, iy)
#                     f_score = tentative_g + heuristic(nix, niy)
#                     heapq.heappush(open_set, (f_score, tentative_g, (nix, niy)))

#         return None  # no path

#     # --- user-facing planning helpers --------------------------------------

#     def plan_path(
#         self,
#         start_xy: Tuple[float, float],
#         goal_xy: Tuple[float, float],
#         inflation_radius_cells: int,
#     ) -> List[Tuple[float, float]]:
#         """Plan a path and return list of (x, y) in world coordinates."""
#         occ_inflated = self.inflate_occ_grid(self.occ_grid, inflation_radius_cells)

#         sx, sy = self.world_to_grid(*start_xy)
#         gx, gy = self.world_to_grid(*goal_xy)

#         path_idx = self.astar(occ_inflated, (sx, sy), (gx, gy))
#         if path_idx is None:
#             raise RuntimeError("A* failed to find a path from start to goal")

#         return [self.grid_to_world(ix, iy) for (ix, iy) in path_idx]

#     def plan_both(
#         self,
#         start_pose: Tuple[float, float, float],
#         goal_pose: Tuple[float, float, float],
#         safety_radius_cells: int,
#         fast_radius_cells: int,
#     ) -> Tuple[List[Tuple[float, float]], List[Tuple[float, float]]]:
#         start_xy = (start_pose[0], start_pose[1])
#         goal_xy = (goal_pose[0], goal_pose[1])
#         safety_path = self.plan_path(start_xy, goal_xy, safety_radius_cells)
#         fast_path = self.plan_path(start_xy, goal_xy, fast_radius_cells)
#         return safety_path, fast_path
    

# class Hw4PlanningNode(Hw2SolutionNode):
#     """
#     HW4 node = HW2 follower + offline global planner.

#     At startup:
#       - Builds a grid from the AprilTag map
#       - Plans a 'safety' and a 'fast' path using A*
#       - Chooses one based on the 'planner_mode' parameter
#       - Loads the chosen path as waypoints for the HW2 controller
#     """

#     def __init__(self):
#         super().__init__()  # Hw2SolutionNode -> Node

#         # Let user choose path type via ROS parameter
#         self.declare_parameter("planner_mode", "safety")
#         mode = (
#             self.get_parameter("planner_mode")
#             .get_parameter_value()
#             .string_value
#         )

#         # Resolve map path (you can make cfg.APRILTAG_MAP_FILE absolute instead)
#         yaml_path = cfg.APRILTAG_MAP_FILE

#         self.get_logger().info(f"Loading AprilTag map from: {yaml_path}")
#         self._planner = OfflineGridPlanner(
#             yaml_path,
#             resolution=cfg.GRID_RESOLUTION,
#             margin=cfg.MAP_MARGIN_M,
#             obstacle_tag_ids=cfg.OBSTACLE_TAG_IDS,
#         )

#         # Offline planning: compute both global paths ONCE at startup
#         safety_path_xy, fast_path_xy = self._planner.plan_both(
#             cfg.DEFAULT_START,
#             cfg.DEFAULT_GOAL,
#             cfg.SAFETY_INFLATION_RADIUS_CELLS,
#             cfg.FAST_INFLATION_RADIUS_CELLS,
#         )

#         # Optionally downsample to fewer waypoints
#         self.safety_path = self._downsample_path(safety_path_xy)
#         self.fast_path = self._downsample_path(fast_path_xy)

#         self.get_logger().info(
#             f"Safety path has {len(self.safety_path)} points, "
#             f"fast path has {len(self.fast_path)} points."
#         )

#         # Choose which to execute
#         if mode == "safety":
#             chosen = self.safety_path
#             self.get_logger().info("Using SAFETY path (max clearance).")
#         elif mode == "fast":
#             chosen = self.fast_path
#             self.get_logger().info("Using FAST path (shorter, closer to obstacle).")
#         else:
#             self.get_logger().warn(
#                 f"Unknown planner_mode '{mode}', defaulting to 'safety'."
#             )
#             chosen = self.safety_path

#         # Convert (x,y) path to (x,y,theta) waypoints and feed HW2 follower
#         self.set_waypoints_from_path(chosen)

#     @staticmethod
#     def _downsample_path(
#         path_xy: List[Tuple[float, float]],
#         step: int = 3,
#     ) -> List[Tuple[float, float]]:
#         """Keep every 'step'-th point to avoid super dense waypoints."""
#         if len(path_xy) <= 2 or step <= 1:
#             return path_xy
#         down = path_xy[::step]
#         if down[-1] != path_xy[-1]:
#             down.append(path_xy[-1])
#         return down

#     def set_waypoints_from_path(
#         self,
#         path_xy: List[Tuple[float, float]],
#     ) -> None:
#         """
#         Convert (x,y) path into (x,y,yaw) waypoints in the map frame and
#         load them into the HW2 follower.

#         This assumes your Hw2SolutionNode exposes 'self.waypoints' and
#         'self.current_waypoint_index' exactly as in HW2. If your names
#         differ, adapt this function accordingly.
#         """
#         if not path_xy:
#             self.get_logger().error("Path is empty; cannot set waypoints.")
#             return

#         waypoints: List[Tuple[float, float, float]] = []
#         for i, (x, y) in enumerate(path_xy):
#             if i < len(path_xy) - 1:
#                 nx, ny = path_xy[i + 1]
#                 yaw = math.atan2(ny - y, nx - x)
#             else:
#                 # Last point: keep yaw pointing roughly toward previous point
#                 if len(path_xy) >= 2:
#                     px, py = path_xy[-2]
#                     yaw = math.atan2(y - py, x - px)
#                 else:
#                     yaw = 0.0
#             waypoints.append((x, y, yaw))

#         # These attributes are inherited from your HW2 solution
#         self.waypoints = waypoints
#         self.current_waypoint_index = 0

#         self.get_logger().info(
#             f"Loaded {len(waypoints)} waypoints into HW2 follower "
#             f"(start at {waypoints[0]}, goal at {waypoints[-1]})."
#         )



# def main(args=None):
#     rclpy.init(args=args)
#     node = Hw4PlanningNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info('HW4 planning node stopped by keyboard interrupt')
#     finally:
#         node.stop_robot()  # inherited from Hw2SolutionNode
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()


# hw4_planning/hw4_planning/planner_node.py
#!/usr/bin/env python3
import math
import heapq
import json
import os
from typing import List, Tuple, Optional, Dict

import numpy as np
import rclpy
from rclpy.node import Node

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None  # type: ignore

# Reuse the HW2 solution node (localization + waypoint follower + PID)
from hw_2_solution.hw2_solution import Hw2SolutionNode  # type: ignore

# HW4 config: DEFAULT_START/GOAL, inflation radii, etc.
from . import config as cfg

import yaml

GridIndex = Tuple[int, int]
WorldPoint = Tuple[float, float]


class Hw4PlanningNode(Hw2SolutionNode):
    """HW4 planning node with offline global planning using AprilTags."""

    def __init__(self) -> None:
        super().__init__()  # sets up timers, PID, AprilTag localization, etc.

        self.get_logger().info("HW4PlanningNode: initialized (extending HW2).")

        # ------------------------------------------------------------------
        # 1) Load AprilTag map (full world model known a priori)
        # ------------------------------------------------------------------
        self.apriltag_map_path = self._resolve_apriltag_map_path()
        self.tags = self._load_apriltag_map(self.apriltag_map_path)

        # Tags we treat as obstacle corners (8–11 by default)
        self.obstacle_tag_ids = getattr(cfg, "OBSTACLE_TAG_IDS", (8, 9, 10, 11))
        self.get_logger().info(
            f"Using obstacle tags {self.obstacle_tag_ids} from "
            f"{os.path.basename(self.apriltag_map_path)}"
        )

        # ------------------------------------------------------------------
        # 2) Start/goal and grid parameters
        # ------------------------------------------------------------------
        # These MUST be in meters, from config.py
        self.default_start = cfg.DEFAULT_START  # (x, y, yaw)
        self.default_goal = cfg.DEFAULT_GOAL    # (x, y, yaw)

        # Grid resolution & world margin (meters)
        self.grid_resolution = getattr(cfg, "GRID_RESOLUTION", 0.02)  # 2 cm
        self.map_margin = getattr(cfg, "MAP_MARGIN_M", 0.20)          # 20 cm

        # Derive world bounds from all tags + start/goal
        (self.world_min_x,
         self.world_max_x,
         self.world_min_y,
         self.world_max_y) = self._compute_world_bounds()

        self.get_logger().info(
            f"World bounds: x=[{self.world_min_x:.3f}, {self.world_max_x:.3f}], "
            f"y=[{self.world_min_y:.3f}, {self.world_max_y:.3f}], "
            f"resolution={self.grid_resolution:.3f} m/cell"
        )

        # Obstacle bounding box from tags 8–11 (with small padding)
        (self.obstacle_min_x,
         self.obstacle_max_x,
         self.obstacle_min_y,
         self.obstacle_max_y) = self._compute_obstacle_bounds()

        self.get_logger().info(
            "Obstacle bounds: "
            f"x=[{self.obstacle_min_x:.3f}, {self.obstacle_max_x:.3f}], "
            f"y=[{self.obstacle_min_y:.3f}, {self.obstacle_max_y:.3f}]"
        )

        # Inflation radii (in grid cells)
        self.safety_inflation_radius = cfg.SAFETY_INFLATION_RADIUS_CELLS
        self.fast_inflation_radius = cfg.FAST_INFLATION_RADIUS_CELLS

        # Planner mode: "safety" (more clearance) or "fast" (shorter path)
        self.declare_parameter("planner_mode", "safety")
        self.planner_mode = (
            self.get_parameter("planner_mode")
            .get_parameter_value()
            .string_value
        )
        if self.planner_mode not in ("safety", "fast"):
            self.get_logger().warn(
                f"Invalid planner_mode '{self.planner_mode}', defaulting to 'safety'"
            )
            self.planner_mode = "safety"

        self.get_logger().info(f"Planner mode: {self.planner_mode}")

        # ------------------------------------------------------------------
        # Offline planning: compute path ONCE at startup
        # ------------------------------------------------------------------
        self.plan_and_set_waypoints()

    # ----------------------------------------------------------------------
    # AprilTag map utilities
    # ----------------------------------------------------------------------

    def _resolve_apriltag_map_path(self) -> str:
        """
        Locate apriltags_position.yaml via param or package share.

        You can override via:
          ros2 run hw4_planning hw4_planning_node \
            --ros-args -p apriltag_map_path:=/full/path/to/apriltags_position.yaml
        """
        # Optional override parameter
        self.declare_parameter("apriltag_map_path", "")
        param_path = (
            self.get_parameter("apriltag_map_path")
            .get_parameter_value()
            .string_value
        )
        if param_path:
            return param_path

        # Default: use package share directory if available
        if get_package_share_directory is not None:
            try:
                share_dir = get_package_share_directory("hw4_planning")
                return os.path.join(share_dir, "configs", "apriltags_position.yaml")
            except Exception:
                self.get_logger().warn(
                    "Could not get package share directory, "
                    "falling back to local 'configs/apriltags_position.yaml'."
                )

        # Fallback: local relative path
        return os.path.join(os.getcwd(), "configs", "apriltags_position.yaml")

    def _load_apriltag_map(self, path: str) -> Dict[int, Dict[str, float]]:
        """Load AprilTag positions from YAML as dict: id -> {x, y, ...}."""
        if not os.path.exists(path):
            raise FileNotFoundError(f"AprilTag map file not found: {path}")

        with open(path, "r") as f:
            data = yaml.safe_load(f)

        out: Dict[int, Dict[str, float]] = {}
        for entry in data.get("apriltags", []):
            tid = int(entry["id"])
            out[tid] = entry
        return out

    def _compute_world_bounds(self) -> Tuple[float, float, float, float]:
        """Compute world min/max bounds from tags + start/goal."""
        xs: List[float] = []
        ys: List[float] = []

        for tag in self.tags.values():
            xs.append(float(tag["x"]))
            ys.append(float(tag["y"]))

        # Include start / goal positions
        xs.extend([self.default_start[0], self.default_goal[0]])
        ys.extend([self.default_start[1], self.default_goal[1]])

        min_x = min(xs) - self.map_margin
        max_x = max(xs) + self.map_margin
        min_y = min(ys) - self.map_margin
        max_y = max(ys) + self.map_margin

        return min_x, max_x, min_y, max_y

    def _compute_obstacle_bounds(self) -> Tuple[float, float, float, float]:
        """Compute obstacle bounding box from obstacle tags (8–11)."""
        obs_tags = []
        for tid in self.obstacle_tag_ids:
            if tid not in self.tags:
                self.get_logger().warn(f"Obstacle tag id {tid} not found in map!")
                continue
            obs_tags.append(self.tags[tid])

        if not obs_tags:
            # No obstacle tags found: put a fake obstacle far away
            big = 1e6
            return big, big + 1.0, big, big + 1.0

        xs = [float(t["x"]) for t in obs_tags]
        ys = [float(t["y"]) for t in obs_tags]

        # Small padding so we don't skim the obstacle
        pad = 0.05  # [m]

        return min(xs) - pad, max(xs) + pad, min(ys) - pad, max(ys) + pad

    # ----------------------------------------------------------------------
    # World / grid utilities
    # ----------------------------------------------------------------------

    def build_occupancy_grid(self) -> np.ndarray:
        """
        Create a binary occupancy grid for the workspace.

        Grid is indexed as [iy, ix], with:
          - 0 = free
          - 1 = occupied (central obstacle region)
        """
        width_m = self.world_max_x - self.world_min_x
        height_m = self.world_max_y - self.world_min_y

        nx = int(math.ceil(width_m / self.grid_resolution)) + 1
        ny = int(math.ceil(height_m / self.grid_resolution)) + 1

        grid = np.zeros((ny, nx), dtype=np.uint8)

        # Mark obstacle cells using the obstacle bounding box from tags 8–11
        for iy in range(ny):
            y = self.world_min_y + iy * self.grid_resolution
            for ix in range(nx):
                x = self.world_min_x + ix * self.grid_resolution

                if (
                    self.obstacle_min_x <= x <= self.obstacle_max_x
                    and self.obstacle_min_y <= y <= self.obstacle_max_y
                ):
                    grid[iy, ix] = 1

        return grid

    @staticmethod
    def inflate_obstacles(grid: np.ndarray, radius: int) -> np.ndarray:
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
        """Convert world (x, y) to integer grid indices (ix, iy)."""
        ix = int(round((x - self.world_min_x) / self.grid_resolution))
        iy = int(round((y - self.world_min_y) / self.grid_resolution))
        return ix, iy

    def grid_to_world(self, ix: int, iy: int) -> WorldPoint:
        """Convert grid indices (ix, iy) back to world (x, y)."""
        x = self.world_min_x + ix * self.grid_resolution
        y = self.world_min_y + iy * self.grid_resolution
        return x, y

    # ----------------------------------------------------------------------
    # A* search on occupancy grid
    # ----------------------------------------------------------------------

    @staticmethod
    def astar(
        occ_grid: np.ndarray,
        start: GridIndex,
        goal: GridIndex,
    ) -> Optional[List[GridIndex]]:
        """
        Run A* on a 2D occupancy grid.

        occ_grid: ny x nx, 0 = free, 1 = occupied
        start / goal: (ix, iy) in grid coordinates
        """
        ny, nx = occ_grid.shape
        sx, sy = start
        gx, gy = goal

        def in_bounds(ix: int, iy: int) -> bool:
            return 0 <= ix < nx and 0 <= iy < ny

        def is_free(ix: int, iy: int) -> bool:
            return occ_grid[iy, ix] == 0

        # 8-connected neighbors
        neighbors = [
            (-1, 0),
            (1, 0),
            (0, -1),
            (0, 1),
            (-1, -1),
            (-1, 1),
            (1, -1),
            (1, 1),
        ]

        def heuristic(ix: int, iy: int) -> float:
            return math.hypot(gx - ix, gy - iy)

        open_set: List[Tuple[float, float, GridIndex]] = []
        heapq.heappush(open_set, (heuristic(sx, sy), 0.0, (sx, sy)))

        came_from: Dict[GridIndex, GridIndex] = {}
        g_score: Dict[GridIndex, float] = {(sx, sy): 0.0}

        while open_set:
            f, g, (ix, iy) = heapq.heappop(open_set)
            if (ix, iy) == (gx, gy):
                # Reconstruct path
                path: List[GridIndex] = [(ix, iy)]
                while (ix, iy) in came_from:
                    ix, iy = came_from[(ix, iy)]
                    path.append((ix, iy))
                path.reverse()
                return path

            for dx, dy in neighbors:
                nix, niy = ix + dx, iy + dy
                if not in_bounds(nix, niy) or not is_free(nix, niy):
                    continue

                step_cost = math.hypot(dx, dy)
                tentative_g = g + step_cost
                if tentative_g < g_score.get((nix, niy), float("inf")):
                    g_score[(nix, niy)] = tentative_g
                    came_from[(nix, niy)] = (ix, iy)
                    f_score = tentative_g + heuristic(nix, niy)
                    heapq.heappush(open_set, (f_score, tentative_g, (nix, niy)))

        return None

    @staticmethod
    def simplify_grid_path(path: List[GridIndex]) -> List[GridIndex]:
        """Remove redundant points that lie on straight segments."""
        if len(path) <= 2:
            return path

        simplified: List[GridIndex] = [path[0]]
        prev_dx = prev_dy = None

        for i in range(1, len(path)):
            x0, y0 = path[i - 1]
            x1, y1 = path[i]
            dx = x1 - x0
            dy = y1 - y0

            if prev_dx is not None and prev_dy is not None:
                if dx != prev_dx or dy != prev_dy:
                    simplified.append(path[i - 1])

            prev_dx, prev_dy = dx, dy

        simplified.append(path[-1])
        return simplified

    # ----------------------------------------------------------------------
    # Planning + HW2 waypoint integration
    # ----------------------------------------------------------------------

    def plan_and_set_waypoints(self) -> None:
        """
        Build occupancy grid, run planner, convert to world-space waypoints,
        and override the HW2 waypoints.

        This is called ONCE at startup (offline global planning).
        """
        # 1. Build base occupancy grid from AprilTag obstacle.
        base_grid = self.build_occupancy_grid()

        # 2. Inflate obstacles depending on planner mode.
        if self.planner_mode == "safety":
            radius = self.safety_inflation_radius
        else:  # "fast"
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

        # 5. Convert grid path to continuous world-space waypoints.
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

            # 5.5 Save waypoints to JSON for debugging / analysis
        try:
            # Convert to a nicer dict format
            waypoints_dict = [
                {
                    "index": i,
                    "x": float(x),
                    "y": float(y),
                    "yaw": float(yaw),
                }
                for i, (x, y, yaw) in enumerate(waypoints_list)
            ]

            # Save next to this file (in the hw4_planning package directory)
            out_dir = os.path.dirname(os.path.abspath(__file__))
            out_path = os.path.join(out_dir, f"hw4_waypoints_{self.planner_mode}.json")

            with open(out_path, "w") as f:
                json.dump(waypoints_dict, f, indent=2)

            self.get_logger().info(
                f"Saved {len(waypoints_dict)} waypoints to '{out_path}'."
            )
        except Exception as e:
            self.get_logger().warn(f"Failed to save waypoints JSON: {e!r}")


        # 6. Override the HW2 solution internals.
        self.waypoints = np.array(waypoints_list, dtype=float)
        self.current_waypoint_idx = 0
        self.waypoint_reached = False
        self.stage = "rotate_to_goal"

        self.get_logger().info(
            f"Planned {len(self.waypoints)} waypoints using mode '{self.planner_mode}'."
        )
        self.get_logger().info(
            f"Start: ({start_x:.3f}, {start_y:.3f}), "
            f"Goal: ({goal_x:.3f}, {goal_y:.3f})"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Node = Hw4PlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("HW4 planning node stopped by keyboard interrupt")
    finally:
        node.stop_robot()  # inherited from Hw2SolutionNode
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
