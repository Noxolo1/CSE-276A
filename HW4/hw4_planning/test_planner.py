#!/usr/bin/env python3
"""
HW4 Planner Testing and Visualization Script

Run this script to:
1. Test A* planner without ROS
2. Visualize occupancy grid
3. Visualize planned path
4. Check path validity

Usage:
    python3 test_planner.py [--mode safety|fast] [--show-grid] [--show-path]
"""

import sys
import math
import argparse
from pathlib import Path

# Add parent directory to path so we can import hw4_planning modules
sys.path.insert(0, str(Path(__file__).parent / "hw4_planning"))

from hw4_planning.astar_planner import AStarPlanner
from hw4_planning import hw4_config as cfg


def test_astar_planner(mode: str = "safety", show_grid: bool = True, show_path: bool = True):
    """
    Test the A* planner with configured world and obstacles.

    Args:
        mode: 'safety' or 'fast'
        show_grid: Print occupancy grid
        show_path: Print path waypoints
    """
    print("\n" + "="*70)
    print("HW4 A* PLANNER TEST")
    print("="*70)

    # Initialize planner
    print("\n[1] Initializing planner...")
    planner = AStarPlanner(
        world_min_x=cfg.WORLD_MIN_X,
        world_max_x=cfg.WORLD_MAX_X,
        world_min_y=cfg.WORLD_MIN_Y,
        world_max_y=cfg.WORLD_MAX_Y,
        grid_resolution=cfg.GRID_RESOLUTION,
    )
    print(f"    Grid: {planner.grid_width} x {planner.grid_height} cells")
    print(f"    Resolution: {cfg.GRID_RESOLUTION} m/cell")
    print(f"    World: ({cfg.WORLD_MIN_X}, {cfg.WORLD_MIN_Y}) to "
          f"({cfg.WORLD_MAX_X}, {cfg.WORLD_MAX_Y})")

    # Build occupancy grid
    print(f"\n[2] Building occupancy grid ({mode} mode)...")
    inflation_radius = (
        cfg.SAFETY_INFLATION_RADIUS_CELLS
        if mode == "safety"
        else cfg.FAST_INFLATION_RADIUS_CELLS
    )
    print(f"    Obstacle inflation: {inflation_radius} cells "
          f"({inflation_radius * cfg.GRID_RESOLUTION:.2f} m)")

    grid = planner.build_occupancy_grid(
        obstacle_centers=cfg.OBSTACLES,
        inflation_radius_cells=inflation_radius,
    )

    occupied_cells = int(grid.sum())
    total_cells = grid.size
    free_pct = 100 * (1 - occupied_cells / total_cells)
    print(f"    Occupied: {occupied_cells} cells ({100*occupied_cells/total_cells:.1f}%)")
    print(f"    Free: {total_cells - occupied_cells} cells ({free_pct:.1f}%)")

    # Visualize grid
    if show_grid:
        print("\n[3] Occupancy Grid Visualization (# = occupied, . = free):")
        print("    " + "-" * 50)
        for row in grid:
            print("    " + "".join("#" if cell else "." for cell in row))
        print("    " + "-" * 50)

    # Plan path
    start = cfg.DEFAULT_START[:2]
    goal = cfg.DEFAULT_GOAL[:2]

    print(f"\n[4] Planning path...")
    print(f"    Start: {start}")
    print(f"    Goal: {goal}")

    path = planner.plan(start, goal, grid)

    if path is None:
        print("    ERROR: No path found!")
        return False

    print(f"    SUCCESS: Path found with {len(path)} waypoints")

    # Display path
    if show_path:
        print("\n[5] Path Waypoints:")
        for i, (x, y) in enumerate(path):
            if i < len(path) - 1:
                nx, ny = path[i + 1]
                yaw = math.atan2(ny - y, nx - x)
                heading_deg = math.degrees(yaw)
            else:
                heading_deg = math.degrees(cfg.DEFAULT_GOAL[2])

            print(f"    {i:2d}: ({x:.3f}, {y:.3f}), "
                  f"heading={heading_deg:6.1f}°")

        # Path statistics
        total_distance = 0.0
        for i in range(len(path) - 1):
            dx = path[i + 1][0] - path[i][0]
            dy = path[i + 1][1] - path[i][1]
            total_distance += math.hypot(dx, dy)

        straight_line = math.hypot(
            goal[0] - start[0],
            goal[1] - start[1]
        )
        ratio = total_distance / straight_line if straight_line > 0 else 0

        print(f"\n    Path Statistics:")
        print(f"      Total distance: {total_distance:.3f} m")
        print(f"      Straight line: {straight_line:.3f} m")
        print(f"      Path ratio: {ratio:.2f}x")

    # Validate path
    print("\n[6] Path Validation:")
    valid = True
    for i, (x, y) in enumerate(path):
        ix, iy = planner.world_to_grid(x, y)
        if not planner.is_valid_grid(ix, iy):
            print(f"    ERROR at waypoint {i}: outside grid bounds")
            valid = False
        elif grid[iy, ix] == 1:
            print(f"    ERROR at waypoint {i}: in occupied cell")
            valid = False

    if valid:
        print("    PASS: All waypoints in free space")

    print("\n" + "="*70)
    return valid


def save_grid_visualization(filename: str = "grid_visualization.txt", mode: str = "safety"):
    """Save ASCII visualization of grid to file."""
    planner = AStarPlanner(
        world_min_x=cfg.WORLD_MIN_X,
        world_max_x=cfg.WORLD_MAX_X,
        world_min_y=cfg.WORLD_MIN_Y,
        world_max_y=cfg.WORLD_MAX_Y,
        grid_resolution=cfg.GRID_RESOLUTION,
    )

    inflation_radius = (
        cfg.SAFETY_INFLATION_RADIUS_CELLS
        if mode == "safety"
        else cfg.FAST_INFLATION_RADIUS_CELLS
    )

    grid = planner.build_occupancy_grid(
        obstacle_centers=cfg.OBSTACLES,
        inflation_radius_cells=inflation_radius,
    )

    with open(filename, "w") as f:
        f.write(f"# HW4 A* Planner Grid Visualization ({mode} mode)\n")
        f.write(f"# Grid size: {grid.shape[1]} x {grid.shape[0]} cells\n")
        f.write(f"# Resolution: {cfg.GRID_RESOLUTION} m/cell\n")
        f.write(f"# # = occupied, . = free\n\n")

        for row in grid:
            f.write("".join("#" if cell else "." for cell in row) + "\n")

    print(f"Grid visualization saved to {filename}")


def main():
    parser = argparse.ArgumentParser(
        description="Test HW4 A* planner",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python3 test_planner.py                    # Test with default settings
    python3 test_planner.py --mode fast         # Test fast mode
    python3 test_planner.py --no-show-grid      # Don't show grid
    python3 test_planner.py --save-grid         # Save grid to file
        """,
    )

    parser.add_argument(
        "--mode",
        choices=["safety", "fast"],
        default="safety",
        help="Planner mode (default: safety)",
    )
    parser.add_argument(
        "--show-grid",
        action="store_true",
        default=True,
        help="Show occupancy grid (default: True)",
    )
    parser.add_argument(
        "--no-show-grid",
        action="store_false",
        dest="show_grid",
        help="Don't show occupancy grid",
    )
    parser.add_argument(
        "--show-path",
        action="store_true",
        default=True,
        help="Show path waypoints (default: True)",
    )
    parser.add_argument(
        "--no-show-path",
        action="store_false",
        dest="show_path",
        help="Don't show path waypoints",
    )
    parser.add_argument(
        "--save-grid",
        action="store_true",
        help="Save grid visualization to file",
    )

    args = parser.parse_args()

    # Run test
    success = test_astar_planner(
        mode=args.mode,
        show_grid=args.show_grid,
        show_path=args.show_path,
    )

    if args.save_grid:
        save_grid_visualization(mode=args.mode)

    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
