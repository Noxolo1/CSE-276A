# # hw4_planning/hw4_planning/config.py
# """
# HW4 Planning configuration for an ~8x8 ft workspace with a 1x1 ft obstacle.

# All dimensions are in meters. This module is kept separate so that you can
# easily tweak the environment description without touching the planner code.
# """

# import math

# FT_TO_M = 0.3048

# # Workspace: 8 x 8 ft (scaled down from 10 x 10)
# WORLD_SIZE_FT = 8.0
# WORLD_SIZE_M = WORLD_SIZE_FT * FT_TO_M          # ≈ 2.4384 m
# WORLD_HALF_M = WORLD_SIZE_M / 2.0               # ≈ 1.2192 m

# # Workspace boundaries (map frame). We assume the map origin is at the center.
# WORLD_MIN_X = -WORLD_HALF_M
# WORLD_MAX_X = WORLD_HALF_M
# WORLD_MIN_Y = -WORLD_HALF_M
# WORLD_MAX_Y = WORLD_HALF_M

# # Central obstacle: 1 x 1 ft
# OBSTACLE_SIZE_FT = 1.0
# OBSTACLE_SIZE_M = OBSTACLE_SIZE_FT * FT_TO_M
# OBSTACLE_HALF_M = OBSTACLE_SIZE_M / 2.0

# # Place obstacle at center of map
# OBSTACLE_CENTER_X = 0.0
# OBSTACLE_CENTER_Y = 0.0

# # Grid resolution (approximate cell decomposition)
# GRID_RESOLUTION = 0.05  # [m] ~5 cm

# # Default start & goal (on diagonal axes, inside boundaries).
# # Adjust these to match your measured Start/Stop marks in the map frame.
# DEFAULT_START = (
#     -WORLD_HALF_M + 0.30,  # x
#     -WORLD_HALF_M + 0.30,  # y
#     0.0                    # yaw [rad]
# )

# DEFAULT_GOAL = (
#     WORLD_HALF_M - 0.30,   # x
#     WORLD_HALF_M - 0.30,   # y
#     0.0                    # yaw [rad]
# )

# # Inflation (in grid cells) used by the two planners
# # "Safety" planner inflates obstacles more to maximize clearance.
# SAFETY_INFLATION_RADIUS_CELLS = 3
# FAST_INFLATION_RADIUS_CELLS = 1


# hw4_planning/hw4_planning/config.py
"""
HW4 Planning configuration.

All dimensions are in meters. This module centralizes the environment
description so the planner code can stay clean.

We assume:
- The map frame origin is at the robot's start position (0 in, 0 in).
- All AprilTags in apriltags_position.yaml are measured off this origin.
- Tags 8–11 are mounted around the obstacle (we treat their bounding box
  as the obstacle region).
"""

import math

# -----------------------------------------------------------
# Unit conversions
# -----------------------------------------------------------
FT_TO_M = 0.3048
IN_TO_M = 0.0254

# -----------------------------------------------------------
# Workspace size (approx; used only for documentation)
# You measured a ~10x10 ft-ish workspace, but we derive bounds
# from the AprilTag map, so these are mostly informational.
# -----------------------------------------------------------
WORLD_SIZE_FT = 8.0          # legacy from HW3; not strictly used
WORLD_SIZE_M = WORLD_SIZE_FT * FT_TO_M
WORLD_HALF_M = WORLD_SIZE_M / 2.0

# -----------------------------------------------------------
# Start / Goal in inches (from HW4 diagram), converted to meters
# Origin (0,0) is your starting location in the map frame.
# -----------------------------------------------------------
START_IN = (0.0, 0.0)        # (x_in, y_in)
GOAL_IN = (65.0, 60.5)       # (x_in, y_in) measured from start

DEFAULT_START = (
    START_IN[0] * IN_TO_M,   # x [m]
    START_IN[1] * IN_TO_M,   # y [m]
    0.0                      # yaw [rad]
)

DEFAULT_GOAL = (
    GOAL_IN[0] * IN_TO_M,    # x [m] ≈ 1.651
    GOAL_IN[1] * IN_TO_M,    # y [m] ≈ 1.5367
    0.0                      # yaw [rad]
)

# -----------------------------------------------------------
# AprilTag map + obstacle description
# -----------------------------------------------------------

# Path to your AprilTag map.  You can make this absolute on the RubikPi
# (e.g. "/home/ubuntu/ros2_ws/rubikpi_ros2/hw4_planning/config/apriltags_position.yaml")
# or keep it relative to your launch working directory.
APRILTAG_MAP_FILE = "apriltags_position.yaml"

# Tags 8-11 are mounted around the obstacle and define the obstacle region.
OBSTACLE_TAG_IDS = (8, 9, 10, 11)

# -----------------------------------------------------------
# Grid representation parameters (for A* planning)
# -----------------------------------------------------------

# Resolution of the occupancy grid in meters/cell
GRID_RESOLUTION = 0.02       # 2 cm cells

# Extra padding around the min/max of all tags/start/goal when building the grid
MAP_MARGIN_M = 0.20          # 20 cm

# How much to "inflate" obstacles (in grid cells) for each planner:
# - Safety planner: more inflation -> more clearance, longer path
# - Fast planner: less inflation -> shorter path, closer to obstacle
SAFETY_INFLATION_RADIUS_CELLS = 5 # was 3
FAST_INFLATION_RADIUS_CELLS = 1
