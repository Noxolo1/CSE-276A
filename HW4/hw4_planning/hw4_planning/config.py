# hw4_planning/hw4_planning/config.py
"""
HW4 Planning configuration for an ~8x8 ft workspace with a 1x1 ft obstacle.

All dimensions are in meters. This module is kept separate so that you can
easily tweak the environment description without touching the planner code.
"""

import math

FT_TO_M = 0.3048

# Workspace: 8 x 8 ft (scaled down from 10 x 10)
WORLD_SIZE_FT = 8.0
WORLD_SIZE_M = WORLD_SIZE_FT * FT_TO_M          # ≈ 2.4384 m
WORLD_HALF_M = WORLD_SIZE_M / 2.0               # ≈ 1.2192 m

# Workspace boundaries (map frame). We assume the map origin is at the center.
WORLD_MIN_X = -WORLD_HALF_M
WORLD_MAX_X = WORLD_HALF_M
WORLD_MIN_Y = -WORLD_HALF_M
WORLD_MAX_Y = WORLD_HALF_M

# Central obstacle: 1 x 1 ft
OBSTACLE_SIZE_FT = 1.0
OBSTACLE_SIZE_M = OBSTACLE_SIZE_FT * FT_TO_M
OBSTACLE_HALF_M = OBSTACLE_SIZE_M / 2.0

# Place obstacle at center of map
OBSTACLE_CENTER_X = 0.0
OBSTACLE_CENTER_Y = 0.0

# Grid resolution (approximate cell decomposition)
GRID_RESOLUTION = 0.05  # [m] ~5 cm

# Default start & goal (on diagonal axes, inside boundaries).
# Adjust these to match your measured Start/Stop marks in the map frame.
DEFAULT_START = (
    -WORLD_HALF_M + 0.30,  # x
    -WORLD_HALF_M + 0.30,  # y
    0.0                    # yaw [rad]
)

DEFAULT_GOAL = (
    WORLD_HALF_M - 0.30,   # x
    WORLD_HALF_M - 0.30,   # y
    0.0                    # yaw [rad]
)

# Inflation (in grid cells) used by the two planners
# "Safety" planner inflates obstacles more to maximize clearance.
SAFETY_INFLATION_RADIUS_CELLS = 3
FAST_INFLATION_RADIUS_CELLS = 1
