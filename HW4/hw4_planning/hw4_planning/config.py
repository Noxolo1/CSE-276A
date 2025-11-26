# assumptions: 
#   the map frame origin is at the robot's start position (0, 0)
#   all apriltags in apriltags_position.yaml are measured off this origin
#   tags 8–11 are mounted around the obstacle (we treat their bounding box as the obstacle region)

import math

FT_TO_M = 0.3048
IN_TO_M = 0.0254

# workspace size was approximately 8x8ft 
WORLD_SIZE_FT = 8.0 
WORLD_SIZE_M = WORLD_SIZE_FT * FT_TO_M
WORLD_HALF_M = WORLD_SIZE_M / 2.0


# start and goal (measured in inches then converted)
START_IN = (0.0, 0.0)        # 
GOAL_IN = (65.0, 60.5)       # measured from start

DEFAULT_START = (
    START_IN[0] * IN_TO_M,   # x (m)
    START_IN[1] * IN_TO_M,   # y (m)
    0.0                      # orientation (rad))
)

DEFAULT_GOAL = (
    GOAL_IN[0] * IN_TO_M,
    GOAL_IN[1] * IN_TO_M,    
    0.0                      
)

APRILTAG_MAP_FILE = "apriltags_position.yaml"

# tags 8-11 define obstacle as a square bounding box
OBSTACLE_TAG_IDS = (8, 9, 10, 11)

# resolution of the occupancy grid in meters/cell
GRID_RESOLUTION = 0.02 # 2 cm cells
MAP_MARGIN_M = 0.20 

# safety path: more inflation -> more clearance, drive longer path
# fast path: less inflation -> shorter path, drive closer to obstacle
SAFETY_INFLATION_RADIUS_CELLS = 18
FAST_INFLATION_RADIUS_CELLS = 1
