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
    get_package_share_directory = None 

# reusing the HW2 solution node (localization + waypoint follower + PID although we ended up making
# separate waypoint follower + PID)
from hw_2_solution.hw2_solution import Hw2SolutionNode

# HW4 configs defined in config.py
from . import config as cfg

import yaml

GridIndex = Tuple[int, int]
WorldPoint = Tuple[float, float]


class Hw4PlanningNode(Hw2SolutionNode):

    def __init__(self) -> None:
        super().__init__()

        self.get_logger().info("HW4PlanningNode initialized")

        # load apriltag map (map defined apriori)
        self.apriltag_map_path = self._resolve_apriltag_map_path()
        self.tags = self._load_apriltag_map(self.apriltag_map_path)

        # obstacle tags (8-11)
        self.obstacle_tag_ids = getattr(cfg, "OBSTACLE_TAG_IDS", (8, 9, 10, 11))
        self.get_logger().info(
            f"Using obstacle tags {self.obstacle_tag_ids} from "
            f"{os.path.basename(self.apriltag_map_path)}"
        )

        # start & goal
        self.default_start = cfg.DEFAULT_START  # (x, y, yaw)
        self.default_goal = cfg.DEFAULT_GOAL    # (x, y, yaw)

        # grid sesolution and map margin
        self.grid_resolution = getattr(cfg, "GRID_RESOLUTION", 0.02)  # 2 cm
        self.map_margin = getattr(cfg, "MAP_MARGIN_M", 0.20)          # 20 cm

        # get world bounds
        (self.world_min_x,
         self.world_max_x,
         self.world_min_y,
         self.world_max_y) = self._compute_world_bounds()

        self.get_logger().info(
            f"World bounds: x=[{self.world_min_x:.3f}, {self.world_max_x:.3f}], "
            f"y=[{self.world_min_y:.3f}, {self.world_max_y:.3f}], "
            f"resolution={self.grid_resolution:.3f} m/cell"
        )

        # bounding box for obstacle
        (self.obstacle_min_x,
         self.obstacle_max_x,
         self.obstacle_min_y,
         self.obstacle_max_y) = self._compute_obstacle_bounds()

        self.get_logger().info(
            "Obstacle bounds: "
            f"x=[{self.obstacle_min_x:.3f}, {self.obstacle_max_x:.3f}], "
            f"y=[{self.obstacle_min_y:.3f}, {self.obstacle_max_y:.3f}]"
        )

        # inflation radius (in grid cells)
        self.safety_inflation_radius = cfg.SAFETY_INFLATION_RADIUS_CELLS
        self.fast_inflation_radius = cfg.FAST_INFLATION_RADIUS_CELLS

        # define path planner mode
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
        
        self.plan_and_set_waypoints()

        self.trajectory_log = []
        self.trajectory_log_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            f"hw4_executed_trajectory_{self.planner_mode}.json",
        )

    # logging helpers 
    def log_trajectory_sample(self) -> None:
        x, y, theta = self.current_state
        t = self.get_clock().now().nanoseconds / 1e9
        entry = {
            "time": float(t),
            "x": float(x),
            "y": float(y),
            "theta": float(theta),
            "using_tag": bool(getattr(self, "using_tag_localization", False)),
        }
        self.trajectory_log.append(entry)

    def save_trajectory_log(self) -> None:
        if not self.trajectory_log:
            return
        try:
            with open(self.trajectory_log_path, "w") as f:
                json.dump(self.trajectory_log, f, indent=2)
            self.get_logger().info(
                f"Saved {len(self.trajectory_log)} samples to '{self.trajectory_log_path}'."
            )
        except Exception as e:
            self.get_logger().warn(f"Failed to save trajectory log: {e!r}")

    # apriltag helper functions
    def _resolve_apriltag_map_path(self) -> str:

        # Optional override parameter
        self.declare_parameter("apriltag_map_path", "")
        param_path = (
            self.get_parameter("apriltag_map_path")
            .get_parameter_value()
            .string_value
        )
        if param_path:
            return param_path

        # use package shared directory when available
        if get_package_share_directory is not None:
            try:
                share_dir = get_package_share_directory("hw4_planning")
                return os.path.join(share_dir, "configs", "apriltags_position.yaml")
            except Exception:
                self.get_logger().warn(
                    "could not find directory"
                )

        return os.path.join(os.getcwd(), "configs", "apriltags_position.yaml")

    def _load_apriltag_map(self, path: str) -> Dict[int, Dict[str, float]]:
        # load apriltag positions as dict
        if not os.path.exists(path):
            raise FileNotFoundError(f"apriltag map file not found: {path}")

        with open(path, "r") as f:
            data = yaml.safe_load(f)

        out: Dict[int, Dict[str, float]] = {}
        for entry in data.get("apriltags", []):
            tid = int(entry["id"])
            out[tid] = entry
        return out

    def _compute_world_bounds(self) -> Tuple[float, float, float, float]:
        # compute world min/max bounds from tags and start/goal
        xs: List[float] = []
        ys: List[float] = []

        for tag in self.tags.values():
            xs.append(float(tag["x"]))
            ys.append(float(tag["y"]))

        # include start and goal positions
        xs.extend([self.default_start[0], self.default_goal[0]])
        ys.extend([self.default_start[1], self.default_goal[1]])

        min_x = min(xs) - self.map_margin
        max_x = max(xs) + self.map_margin
        min_y = min(ys) - self.map_margin
        max_y = max(ys) + self.map_margin

        return min_x, max_x, min_y, max_y

    def _compute_obstacle_bounds(self) -> Tuple[float, float, float, float]:
        #  compute obstacle bounding box from obstacle tags
        obs_tags = []
        for tid in self.obstacle_tag_ids:
            if tid not in self.tags:
                self.get_logger().warn(f"Obstacle tag id {tid} not found in map!")
                continue
            obs_tags.append(self.tags[tid])

        if not obs_tags:
            # make a big fake obstacle if no tags found 
            big = 1e6
            return big, big + 1.0, big, big + 1.0

        xs = [float(t["x"]) for t in obs_tags]
        ys = [float(t["y"]) for t in obs_tags]

        pad = 0.05  

        return min(xs) - pad, max(xs) + pad, min(ys) - pad, max(ys) + pad


    def build_occupancy_grid(self) -> np.ndarray:
        
        # creates a binary occupancy grid indexed as [iy, ix], with:
        #  0 = free
        #  1 = occupied (central obstacle region)
        
        width_m = self.world_max_x - self.world_min_x
        height_m = self.world_max_y - self.world_min_y

        nx = int(math.ceil(width_m / self.grid_resolution)) + 1
        ny = int(math.ceil(height_m / self.grid_resolution)) + 1

        grid = np.zeros((ny, nx), dtype=np.uint8)

        # mark obstacle cells using the obstacle bounding box from tags 8–11
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
        # inflate obstacles by a Chebyshev radius (in grid cells)
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
        # converts world (x, y) to integer grid indices (ix, iy) 
        ix = int(round((x - self.world_min_x) / self.grid_resolution))
        iy = int(round((y - self.world_min_y) / self.grid_resolution))
        return ix, iy

    def grid_to_world(self, ix: int, iy: int) -> WorldPoint:
        # converts grid indices (ix, iy) back to world (x, y)
        x = self.world_min_x + ix * self.grid_resolution
        y = self.world_min_y + iy * self.grid_resolution
        return x, y

    
    # astar search on occupancy grid
    @staticmethod
    def astar(
        occ_grid: np.ndarray,
        start: GridIndex,
        goal: GridIndex,
    ) -> Optional[List[GridIndex]]:
    
        #occ_grid: ny x nx, 0 = free, 1 = occupied
        # start/goal: (ix, iy) in grid coordinates
        ny, nx = occ_grid.shape
        sx, sy = start
        gx, gy = goal

        def in_bounds(ix: int, iy: int) -> bool:
            return 0 <= ix < nx and 0 <= iy < ny

        def is_free(ix: int, iy: int) -> bool:
            return occ_grid[iy, ix] == 0

        # 8 connected neighbors
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
        # remove redundant points that lie on straight segments to decrease
        # number of points of path
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


    def plan_and_set_waypoints(self) -> None:
        
        # build occupancy grid and run planner 
        # gets called once at startup (offline global planning)
        
        base_grid = self.build_occupancy_grid()

        # inflate obstacles depending on mode chosen 
        if self.planner_mode == "safety":
            radius = self.safety_inflation_radius # safety 
        else: 
            radius = self.fast_inflation_radius # fast 
 
        occ_grid = self.inflate_obstacles(base_grid, radius)

        # change start/goal in grid indices
        start_x, start_y, start_yaw = self.default_start
        goal_x, goal_y, goal_yaw = self.default_goal

        sx, sy = self.world_to_grid(start_x, start_y)
        gx, gy = self.world_to_grid(goal_x, goal_y)

        ny, nx = occ_grid.shape
        if not (0 <= sx < nx and 0 <= sy < ny):
            self.get_logger().error("start cell is out of bounds")
            return
        if not (0 <= gx < nx and 0 <= gy < ny):
            self.get_logger().error("goal cell is out of bounds")
            return

        if occ_grid[sy, sx] == 1:
            self.get_logger().error("start cell is in obstacle")
            return
        if occ_grid[gy, gx] == 1:
            self.get_logger().error("goal cell is in obstacle")
            return

        # run astar
        grid_path = self.astar(occ_grid, (sx, sy), (gx, gy))
        if grid_path is None:
            self.get_logger().error("astar failed to find a path")
            return

        grid_path = self.simplify_grid_path(grid_path)
        self.get_logger().info(f"Grid path has {len(grid_path)} grid points")

        # convert grid to wrold coordinates
        waypoints_list: List[List[float]] = []
        for i, (ix, iy) in enumerate(grid_path):
            x, y = self.grid_to_world(ix, iy)

            if i < len(grid_path) - 1:
                nx_i, ny_i = grid_path[i + 1]
                nx_w, ny_w = self.grid_to_world(nx_i, ny_i)
                yaw = math.atan2(ny_w - y, nx_w - x)
            else:
                # final orientation (uses goal yaw directly)
                yaw = goal_yaw

            waypoints_list.append([x, y, yaw])

        # downsample waypoints for easier robot navigation 
        target_waypoints = 14
        if len(waypoints_list) > target_waypoints:
            idxs = np.linspace(0, len(waypoints_list) - 1, target_waypoints)
            idxs = np.round(idxs).astype(int)
            waypoints_list = [waypoints_list[i] for i in idxs]

        self.get_logger().info(
            f"Using {len(waypoints_list)} waypoints after downsampling "
            f"(target={target_waypoints})."
        )

        # logging
        try:
            waypoints_dict = [
                {
                    "index": i,
                    "x": float(x),
                    "y": float(y),
                    "yaw": float(yaw),
                }
                for i, (x, y, yaw) in enumerate(waypoints_list)
            ]

            out_dir = os.path.dirname(os.path.abspath(__file__))
            out_path = os.path.join(
                out_dir, f"hw4_waypoints_{self.planner_mode}.json"
            )

            with open(out_path, "w") as f:
                json.dump(waypoints_dict, f, indent=2)

            self.get_logger().info(
                f"Saved {len(waypoints_dict)} waypoints to '{out_path}'."
            )
        except Exception as e:
            self.get_logger().warn(f"failed to save waypoints JSON: {e!r}")

        # override hw2 solution
        self.waypoints = np.array(waypoints_list, dtype=float)
        self.current_waypoint_idx = 0
        self.waypoint_reached = False
        self.stage = "rotate_to_goal"

        self.get_logger().info(
            f"planned {len(self.waypoints)} waypoints using mode '{self.planner_mode}'."
        )
        self.get_logger().info(
            f"start: ({start_x:.3f}, {start_y:.3f}), "
            f"goal: ({goal_x:.3f}, {goal_y:.3f})"
        )



def main(args=None) -> None:
    rclpy.init(args=args)
    node: Node = Hw4PlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("HW4 planning node stopped by keyboard interrupt")
    finally:
        node.stop_robot()
        node.save_trajectory_log()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
