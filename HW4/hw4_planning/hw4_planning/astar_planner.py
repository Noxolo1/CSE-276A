"""
A* Path Planning Algorithm

This module implements a grid-based A* path planner for 2D workspace navigation.
Supports multiple inflation strategies (safety vs fast) for obstacle avoidance.
"""

import heapq
import math
from typing import List, Tuple, Optional, Set
import numpy as np


GridIndex = Tuple[int, int]
WorldPoint = Tuple[float, float]
Path = List[Tuple[float, float]]


class AStarPlanner:
    """
    A* grid-based path planner for 2D environments with rectangular obstacles.
    
    Converts world coordinates to grid cells and performs A* search with
    configurable obstacle inflation for safety vs. speed trade-off.
    """

    def __init__(
        self,
        world_min_x: float,
        world_max_x: float,
        world_min_y: float,
        world_max_y: float,
        grid_resolution: float,
    ):
        """
        Initialize the planner with world boundaries and grid resolution.

        Args:
            world_min_x, world_max_x: X bounds of the workspace (meters)
            world_min_y, world_max_y: Y bounds of the workspace (meters)
            grid_resolution: Size of each grid cell (meters)
        """
        self.world_min_x = world_min_x
        self.world_max_x = world_max_x
        self.world_min_y = world_min_y
        self.world_max_y = world_max_y
        self.grid_resolution = grid_resolution

        width_m = world_max_x - world_min_x
        height_m = world_max_y - world_min_y

        self.grid_width = int(round(width_m / grid_resolution)) + 1
        self.grid_height = int(round(height_m / grid_resolution)) + 1

    def world_to_grid(self, x: float, y: float) -> GridIndex:
        """Convert world coordinates to grid indices."""
        ix = int(round((x - self.world_min_x) / self.grid_resolution))
        iy = int(round((y - self.world_min_y) / self.grid_resolution))
        return (ix, iy)

    def grid_to_world(self, ix: int, iy: int) -> WorldPoint:
        """Convert grid indices to world coordinates (center of cell)."""
        x = self.world_min_x + ix * self.grid_resolution
        y = self.world_min_y + iy * self.grid_resolution
        return (x, y)

    def is_valid_grid(self, ix: int, iy: int) -> bool:
        """Check if grid cell is within bounds."""
        return 0 <= ix < self.grid_width and 0 <= iy < self.grid_height

    def build_occupancy_grid(
        self,
        obstacle_centers: List[Tuple[float, float, float, float]],
        inflation_radius_cells: int = 0,
    ) -> np.ndarray:
        """
        Build binary occupancy grid with optional obstacle inflation.

        Args:
            obstacle_centers: List of (x, y, half_width, half_height) rectangles
            inflation_radius_cells: Expand obstacles by this many cells

        Returns:
            Grid array: 0 = free, 1 = occupied (shape: [height, width])
        """
        grid = np.zeros((self.grid_height, self.grid_width), dtype=np.uint8)

        for ox, oy, hw, hh in obstacle_centers:
            # Find grid cells occupied by rectangle
            min_x = max(0, int((ox - hw - self.world_min_x) / self.grid_resolution))
            max_x = min(
                self.grid_width - 1,
                int((ox + hw - self.world_min_x) / self.grid_resolution) + 1,
            )
            min_y = max(0, int((oy - hh - self.world_min_y) / self.grid_resolution))
            max_y = min(
                self.grid_height - 1,
                int((oy + hh - self.world_min_y) / self.grid_resolution) + 1,
            )

            grid[min_y : max_y + 1, min_x : max_x + 1] = 1

        # Inflate obstacles if requested
        if inflation_radius_cells > 0:
            from scipy import ndimage

            grid = ndimage.binary_dilation(
                grid, iterations=inflation_radius_cells
            ).astype(np.uint8)

        return grid

    def get_neighbors(
        self, cell: GridIndex, grid: np.ndarray
    ) -> List[Tuple[GridIndex, float]]:
        """
        Get valid neighboring cells (8-connected) with movement costs.

        Args:
            cell: Current grid cell
            grid: Occupancy grid

        Returns:
            List of (neighbor_cell, cost) tuples
        """
        ix, iy = cell
        neighbors = []

        # 8-connected neighbors: cardinal + diagonal
        deltas = [
            (0, 1, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (-1, 0, 1.0),
            (1, 1, math.sqrt(2)),
            (1, -1, math.sqrt(2)),
            (-1, 1, math.sqrt(2)),
            (-1, -1, math.sqrt(2)),
        ]

        for dx, dy, cost in deltas:
            nix, niy = ix + dx, iy + dy
            if self.is_valid_grid(nix, niy) and grid[niy, nix] == 0:
                neighbors.append(((nix, niy), cost))

        return neighbors

    def heuristic(self, cell: GridIndex, goal: GridIndex) -> float:
        """
        Compute heuristic cost (Euclidean distance in grid cells).

        Args:
            cell: Current cell
            goal: Goal cell

        Returns:
            Heuristic cost estimate
        """
        ix, iy = cell
        gx, gy = goal
        dx = abs(gx - ix)
        dy = abs(gy - iy)
        return math.sqrt(dx * dx + dy * dy)

    def plan(
        self,
        start: WorldPoint,
        goal: WorldPoint,
        grid: np.ndarray,
    ) -> Optional[Path]:
        """
        Plan a path from start to goal using A* algorithm.

        Args:
            start: (x, y) start position in world coordinates
            goal: (x, y) goal position in world coordinates
            grid: Occupancy grid (0=free, 1=occupied)

        Returns:
            List of waypoints (x, y) from start to goal, or None if no path exists
        """
        start_grid = self.world_to_grid(start[0], start[1])
        goal_grid = self.world_to_grid(goal[0], goal[1])

        # Validate start and goal
        if not self.is_valid_grid(*start_grid) or grid[start_grid[1], start_grid[0]] == 1:
            return None
        if not self.is_valid_grid(*goal_grid) or grid[goal_grid[1], goal_grid[0]] == 1:
            return None

        # A* search
        open_set: List[Tuple[float, int, GridIndex]] = []
        close_set: Set[GridIndex] = set()
        came_from = {}
        g_score = {start_grid: 0.0}

        counter = 0  # Tie-breaker for heap
        heapq.heappush(
            open_set,
            (
                self.heuristic(start_grid, goal_grid),
                counter,
                start_grid,
            ),
        )

        while open_set:
            _, _, current = heapq.heappop(open_set)

            if current == goal_grid:
                # Reconstruct path
                path = []
                node = goal_grid
                while node in came_from:
                    path.append(self.grid_to_world(node[0], node[1]))
                    node = came_from[node]
                path.append(self.grid_to_world(start_grid[0], start_grid[1]))
                path.reverse()
                return path

            if current in close_set:
                continue

            close_set.add(current)

            for neighbor, cost in self.get_neighbors(current, grid):
                if neighbor in close_set:
                    continue

                tentative_g = g_score[current] + cost
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbor, goal_grid)
                    counter += 1
                    heapq.heappush(open_set, (f_score, counter, neighbor))

        return None  # No path found
