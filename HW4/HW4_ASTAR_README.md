# HW4: A* Path Planning and Robot Navigation

## Overview

This assignment implements a complete autonomous navigation pipeline using the **A* path planning algorithm** combined with **waypoint-based robot control** from HW2.

### Key Features

- **A* Path Planner**: Grid-based pathfinding with obstacle inflation
- **Dual Mode Operation**: 
  - `safety` mode: inflated obstacles for maximum clearance
  - `fast` mode: minimal inflation for shortest paths
- **Integrated Control**: Reuses proportional guidance laws from HW2
- **Robust Localization**: AprilTag-based pose estimation
- **State Machine**: Handles pose loss with search/recovery behavior
- **Independent Package**: Runs completely separately from HW2 & HW3

---

## Architecture

### Module Organization

```
hw4_planning/
├── hw4_planning/
│   ├── astar_planner.py           # A* algorithm implementation
│   ├── hw4_config.py              # Configuration (world, obstacles, gains)
│   ├── planning_node.py           # Main orchestrator
│   ├── waypoint_follower_hw4.py   # Robot control (from HW2 concepts)
│   └── localization_hw4.py        # Pose estimation
├── launch/
│   └── hw4_astar_planning.launch.py
└── configs/
    └── apriltags_position.yaml
```

### Component Responsibilities

#### 1. **AStarPlanner** (`astar_planner.py`)
- Converts world coordinates to grid cells
- Builds occupancy grid with optional obstacle inflation
- Implements A* search with heuristic
- Returns waypoint path in world coordinates

#### 2. **Configuration** (`hw4_config.py`)
- World boundaries: 8 ft × 8 ft (0-2.44 m)
- Central obstacle: 2 ft × 2 ft wooden structure
- Grid resolution: 5 cm per cell
- Robot control gains and limits (from HW2)
- AprilTag map for localization

#### 3. **Planning Node** (`planning_node.py`)
- Initializes A* planner
- Builds occupancy grid
- Plans path from start to goal
- Converts path → waypoints with heading information
- Launches waypoint follower

#### 4. **Waypoint Follower** (`waypoint_follower_hw4.py`)
- Follows waypoints using proportional guidance
- Implements state machine: TRACKING → COAST → SEARCH → FAILSAFE
- Handles pose loss gracefully
- Logs execution to CSV

#### 5. **Localization** (`localization_hw4.py`)
- Subscribes to AprilTag detections
- Computes robot pose relative to known tags
- Publishes pose on `/pose_estimated` topic

---

## Algorithm Details

### A* Search

**Heuristic**: Euclidean distance in grid cells
$$h(n) = \sqrt{(goal_x - node_x)^2 + (goal_y - node_y)^2}$$

**Movement Cost**:
- Cardinal (up/down/left/right): cost = 1.0
- Diagonal (corner): cost = √2 ≈ 1.414

**Obstacle Inflation**:
- **Safety mode**: 3-cell radius → ~15 cm safety margin
- **Fast mode**: 1-cell radius → ~5 cm safety margin

### Robot Control

**Proportional Guidance Law**:
$$v = K_v \cdot \rho$$
$$\omega = K_w \cdot e_{yaw}$$

Where:
- $\rho$ = distance to goal (m)
- $e_{yaw}$ = heading error (rad)
- $K_v = 0.6$ (distance gain)
- $K_w = 1.2$ (heading gain)

**Differential Drive Kinematics**:
$$v_L = v - \frac{1}{2}\omega \cdot baseline$$
$$v_R = v + \frac{1}{2}\omega \cdot baseline$$

Where $baseline = 0.127$ m (wheel track width)

---

## How to Run

### Prerequisites
```bash
sudo apt install python3-scipy python3-numpy ros2-tf-transformations
```

### Build
```bash
cd ~/colcon_ws
colcon build --packages-select hw4_planning
source install/setup.bash
```

### Launch (Safety Mode)
```bash
ros2 launch hw4_planning hw4_astar_planning.launch.py planner_mode:=safety
```

### Launch (Fast Mode)
```bash
ros2 launch hw4_planning hw4_astar_planning.launch.py planner_mode:=fast
```

### Run Individual Nodes
```bash
# Planner only
ros2 run hw4_planning planning_node

# Waypoint follower only (requires external pose estimation)
ros2 run hw4_planning waypoint_follower_hw4

# Localization only
ros2 run hw4_planning localization_hw4_node
```

---

## Configuration

Edit `hw4_config.py` to modify:

```python
# World
WORLD_MIN_X, WORLD_MAX_X = 0.0, 2.44    # Workspace bounds
WORLD_MIN_Y, WORLD_MAX_Y = 0.0, 2.44

# Obstacles
OBSTACLE_CENTER_X, OBSTACLE_CENTER_Y = 1.22, 1.22  # Center position
OBSTACLE_HALF_M = 0.305                              # Half-width/height

# Planning
GRID_RESOLUTION = 0.05                    # Cell size in meters
SAFETY_INFLATION_RADIUS_CELLS = 3         # Safety margin (cells)
FAST_INFLATION_RADIUS_CELLS = 1           # Fast margin (cells)

# Control
MAX_V, MAX_W = 0.2, 0.80                  # Velocity limits
Kv, Kw = 0.6, 1.2                         # Control gains
POS_TOL, YAW_TOL = 0.12, 10°              # Tolerances

# Defaults
DEFAULT_START = (0.2, 0.2, 0.0)           # (x, y, yaw)
DEFAULT_GOAL = (2.2, 2.2, 0.0)
```

---

## Output and Logging

### Occupancy Grid
File: `~/occupancy_grid.txt`
- Text representation of grid
- 0 = free cell, 1 = occupied cell
- Includes metadata (size, resolution, mode)

### Execution Log
File: `~/hw4_log.csv`

Columns:
| t | state | x | y | yaw | gx | gy | gyaw | rho | eyaw | L | R |
|---|-------|---|---|-----|----|----|------|-----|------|---|---|
| Time | Current state | Position X | Position Y | Orientation | Goal X | Goal Y | Goal yaw | Distance to goal | Heading error | Left motor | Right motor |

States: TRACKING, COAST, SEARCH, FAILSAFE

---

## Key Differences from HW2

| Aspect | HW2 | HW4 |
|--------|-----|-----|
| Waypoints | Pre-defined, hardcoded | Generated by A* planner |
| Planning | None (manual) | Autonomous A* grid search |
| Obstacle Avoidance | N/A | Configurable inflation |
| Mode | Single | Dual (safety/fast) |
| Package Dependency | Standalone | Independent from HW2 |

---

## State Machine

```
                   pose fresh?
                   /         \
                  Y           N
                  |           |
              TRACKING ----> COAST
                  ^           |
                  |           | (coast_time)
                  |           |
              (fresh poses)  SEARCH
              (hold_time)     |  |
                  ^           |  | (search_time)
                  |           |  |
                  |           v  |
                  +---<----- FAILSAFE
                             |
                      (pose fresh?)
                             |
                             v
                         TRACKING
```

**State Descriptions**:
- **TRACKING**: Normal waypoint following with fresh pose
- **COAST**: Brief stop while waiting for pose
- **SEARCH**: Rotate in place to regain pose/visual lock
- **FAILSAFE**: Stopped, waits for pose recovery

---

## Troubleshooting

### No Path Found
- Check obstacle configuration
- Verify start/goal are not in obstacles
- Try `fast` mode for more permissive inflation

### Robot Not Following Waypoints
- Verify pose estimation is working (check `/pose_estimated` topic)
- Check motor commands published to correct topic
- Adjust control gains (Kv, Kw) if movement is too aggressive/sluggish

### Poor Path Quality
- Increase `GRID_RESOLUTION` for finer paths
- Use `safety` mode for cleaner routes
- Check AprilTag positions in `MAP_TAGS`

---

## Code Structure Notes

This package is designed to be **completely independent**:
- No imports from HW2, HW3, or other assignments
- All necessary functionality self-contained
- Can be built/run separately in a fresh ROS2 workspace
- Configuration centralized in `hw4_config.py`

This modularity allows you to:
1. Test HW4 independently
2. Swap components (e.g., different planner, controller)
3. Reuse in future projects without conflicts
