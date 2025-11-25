"""
HW4 Path Planning Configuration

Defines world boundaries, obstacles, planning parameters, and control gains
specific to this assignment's environment.
"""

# ============================================================================
# WORLD CONFIGURATION (8ft x 8ft room)
# ============================================================================

WORLD_MIN_X = 0.0  # meters
WORLD_MAX_X = 2.44  # 8 feet
WORLD_MIN_Y = 0.0
WORLD_MAX_Y = 2.44  # 8 feet

# ============================================================================
# OBSTACLE CONFIGURATION
# ============================================================================
# Central obstacle: approximate 2ft x 2ft wooden structure
# (x_center, y_center, half_width, half_height)

OBSTACLE_CENTER_X = 1.22  # center of room
OBSTACLE_CENTER_Y = 1.22
OBSTACLE_HALF_M = 0.305  # ~1 ft

OBSTACLES = [
    (OBSTACLE_CENTER_X, OBSTACLE_CENTER_Y, OBSTACLE_HALF_M, OBSTACLE_HALF_M),
]

# ============================================================================
# GRID-BASED PLANNING PARAMETERS
# ============================================================================

GRID_RESOLUTION = 0.05  # cell size in meters (5 cm)

# Obstacle inflation (safety margin in number of cells)
SAFETY_INFLATION_RADIUS_CELLS = 3  # ~15 cm margin
FAST_INFLATION_RADIUS_CELLS = 1  # ~5 cm margin

# ============================================================================
# DEFAULT START AND GOAL
# ============================================================================

DEFAULT_START = (0.2, 0.2, 0.0)  # (x, y, yaw) in meters and radians
DEFAULT_GOAL = (2.2, 2.2, 0.0)

# ============================================================================
# ROBOT CONTROL PARAMETERS (reused from HW2)
# ============================================================================

POS_TOL = 0.12  # m, position tolerance
YAW_TOL = 10 * 3.14159 / 180  # rad, heading tolerance

# PID/control gains
Kv = 0.6  # proportional gain for distance
Kw = 1.2  # proportional gain for heading

BASELINE = 0.127  # m, wheel track width
K_WHEEL = 1.0  # scaling factor for motor commands

MAX_V = 0.2  # m/s, max linear velocity
MAX_W = 0.80  # rad/s, max angular velocity

# Pose estimation parameters
POSE_STALE_S = 0.50  # consider pose stale after this (seconds)
TIMER_PERIOD_S = 0.05  # control loop frequency (20 Hz)

# Search/recovery parameters
COAST_TIME_S = 0.20  # grace stop before scanning
SEARCH_OMEGA = 0.60  # rad/s, scan rotation speed
REACQUIRE_HOLD_S = 0.15  # require fresh poses for this long
MAX_SEARCH_TIME_S = 8.0  # max time searching for pose

# ============================================================================
# APRIL TAG MAP (from HW2)
# ============================================================================

MAP_TAGS = {
    "tag_0": (0.73330, -0.67310, 0.1645, 0.0),
    "tag_1": (1.29210, 0.00000, 0.1645, 180.0),
    "tag_2": (1.72390, 1.69520, 0.1695, 180.0),
    "tag_3": (1.00000, 2.68580, 0.1645, 270.0),
    "tag_4": (-0.19380, 1.61900, 0.1645, 0.0),
}
