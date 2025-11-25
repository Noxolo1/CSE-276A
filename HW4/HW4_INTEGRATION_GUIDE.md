# HW4 Integration Guide

## Overview

This document explains how the HW4 A* planner integrates with HW2's robot control infrastructure while maintaining complete independence.

---

## Design Philosophy

### Self-Contained Package
- **No external dependencies** on HW2, HW3, or other homeworks
- **All required components** included in `hw4_planning/`
- **Isolated configuration** in `hw4_config.py`
- **Independent launch** via `hw4_astar_planning.launch.py`

### Reused Concepts (Not Code)
While HW4 doesn't import from HW2, it **reuses the same concepts**:

| Concept | HW2 Implementation | HW4 Implementation |
|---------|-------------------|-------------------|
| Robot model | Differential drive kinematics | Same equations in `waypoint_follower_hw4.py` |
| Control law | Proportional guidance | Reimplemented in `compute_tracking_cmd()` |
| Pose estimation | AprilTag-based | `localization_hw4.py` |
| State machine | TRACKING/COAST/SEARCH/FAILSAFE | Same states in follower |
| Motor interface | `Float32MultiArray` on `motor_commands` | Identical topic/message type |

---

## Integration Points

### 1. Localization Module
**File**: `localization_hw4.py`

Provides pose estimation by:
- Subscribing to AprilTag detections (from `rubikpi_ros2` package)
- Computing robot pose relative to world frame
- Publishing on `/pose_estimated` (PoseStamped)

**Required Topics**:
- Input: `/apriltag_pose` (or adapt to your AprilTag package)
- Output: `/pose_estimated` (PoseStamped)

### 2. Planning Module
**File**: `planning_node.py`

Orchestrates the planning pipeline:
```python
# 1. Initialize A* planner
planner = AStarPlanner(...)

# 2. Build occupancy grid with configured obstacles
grid = planner.build_occupancy_grid(cfg.OBSTACLES, inflation_radius)

# 3. Plan path from start to goal
path = planner.plan(start_pos, goal_pos, grid)

# 4. Convert path to waypoints with heading
waypoints = path_to_waypoints(path)

# 5. Launch waypoint follower with generated waypoints
follower = WaypointFollowerHW4(waypoints=waypoints)
```

### 3. Waypoint Following Module
**File**: `waypoint_follower_hw4.py`

Reimplements HW2's waypoint follower logic:
- Tracks waypoints using proportional guidance
- Manages state transitions (TRACKING → COAST → SEARCH → FAILSAFE)
- Publishes motor commands to `/motor_commands` topic
- Subscribes to `/pose_estimated` for pose feedback

**Key Equations**:
$$v = K_v \cdot \rho \quad \text{(distance to goal)}$$
$$\omega = K_w \cdot e_{yaw} \quad \text{(heading error)}$$

**Differential Drive**:
$$L = K_{wheel} \cdot (v - \frac{1}{2}\omega \cdot baseline)$$
$$R = K_{wheel} \cdot (v + \frac{1}{2}\omega \cdot baseline)$$

### 4. Configuration Module
**File**: `hw4_config.py`

Central configuration with:
- World bounds: 8 ft × 8 ft workspace
- Obstacle definition: central 2 ft × 2 ft structure
- Planning parameters: grid resolution, inflation radii
- Control gains: Kv, Kw, velocity limits (identical to HW2)
- AprilTag map: tag positions for localization

**How to Adapt**:
```python
# Change world size
WORLD_MIN_X, WORLD_MAX_X = 0.0, 3.0  # Now 10 feet
WORLD_MIN_Y, WORLD_MAX_Y = 0.0, 3.0

# Add new obstacle (center_x, center_y, half_width, half_height)
OBSTACLES = [
    (1.22, 1.22, 0.305, 0.305),  # Existing
    (2.0, 0.5, 0.2, 0.1),        # New obstacle
]

# Adjust safety/speed trade-off
SAFETY_INFLATION_RADIUS_CELLS = 5    # More conservative
FAST_INFLATION_RADIUS_CELLS = 0      # Aggressive
```

---

## Message Flow

```
┌─────────────────────┐
│ AprilTag Detection  │ (from rubikpi_ros2)
└──────────┬──────────┘
           │ /apriltag_pose (PoseStamped)
           │
    ┌──────▼──────────────┐
    │  Localization Node  │ (localization_hw4.py)
    │   hw4_localization  │
    └──────┬──────────────┘
           │ /pose_estimated (PoseStamped)
           │
    ┌──────▼──────────────────┐
    │  Planning Node          │ (planning_node.py)
    │ • Runs A* planner       │
    │ • Generates path        │
    │ • Launches follower     │
    └──────┬──────────────────┘
           │
    ┌──────▼────────────────────────┐
    │ Waypoint Follower             │ (waypoint_follower_hw4.py)
    │ • Subscribes /pose_estimated  │
    │ • Publishes motor_commands    │
    └──────┬────────────────────────┘
           │ /motor_commands (Float32MultiArray)
           │
    ┌──────▼──────────────────┐
    │  Motor Controller Node  │ (motor_control.py)
    │   hw4_motor_controller  │
    └─────────────────────────┘
```

---

## How to Extend

### Add a New Obstacle

In `hw4_config.py`:
```python
OBSTACLES = [
    (1.22, 1.22, 0.305, 0.305),  # Original
    (x_center, y_center, half_width, half_height),  # New
]
```

### Change Planning Strategy

In `planning_node.py`, modify `_path_to_waypoints()`:
```python
def _path_to_waypoints(self, path):
    # Current: use raw path points
    # Could instead:
    # - Downsample path: keep_every_nth_point()
    # - Smooth path: bezier_spline()
    # - Add intermediate waypoints: insert_intermediate_poses()
    
    waypoints = []
    for i, (x, y) in enumerate(path):
        # ... compute heading ...
        waypoints.append((x, y, yaw))
    return waypoints
```

### Swap Planners

Replace A* with RRT*, PRM, or Dijkstra:
```python
# In planning_node.py
from .my_new_planner import MyNewPlanner

path = MyNewPlanner.plan(start, goal, grid)
```

### Modify Control Behavior

In `waypoint_follower_hw4.py`:
```python
# Change control law from proportional to PID:
self.integral_error += error
output = Kp * error + Ki * self.integral_error + Kd * (error - self.prev_error)
```

---

## Testing Without ROS

Run the standalone test script:

```bash
cd HW4/hw4_planning

# Test planning (requires numpy only)
python3 test_planner.py

# Test different modes
python3 test_planner.py --mode fast
python3 test_planner.py --mode safety

# Save grid visualization
python3 test_planner.py --save-grid

# Quiet mode (no visualization)
python3 test_planner.py --no-show-grid --no-show-path
```

---

## Comparison: HW2 vs HW4

### HW2 Workflow
```
1. Hardcode waypoints
2. Run waypoint follower
3. Robot follows waypoints
4. Done (no planning)
```

### HW4 Workflow
```
1. Configure world & obstacles
2. Run planning node
   ├─ A* generates optimal path
   ├─ Convert path to waypoints
   ├─ Launch waypoint follower
3. Waypoint follower executes plan
4. Done (with planning!)
```

### Key Differences

| Aspect | HW2 | HW4 |
|--------|-----|-----|
| **Planning** | Manual (hardcoded) | Automated (A*) |
| **Flexibility** | Fixed waypoints | Dynamic paths |
| **Obstacles** | Ignored | Respected |
| **Safety** | Up to operator | Configurable |
| **Scalability** | One path | Any start→goal |
| **Package Structure** | Minimal | Self-contained |

---

## Troubleshooting Integration

### Robot Not Following Path

**Check**:
1. Is localization publishing to `/pose_estimated`?
   ```bash
   ros2 topic echo /pose_estimated
   ```

2. Is planner publishing correct waypoints?
   ```bash
   # Check logs during planning_node startup
   ros2 run hw4_planning planning_node
   ```

3. Are motor commands being published?
   ```bash
   ros2 topic echo /motor_commands
   ```

**Fix**:
- Verify AprilTag detections working
- Check tag positions in `hw4_config.py` match physical setup
- Adjust control gains (Kv, Kw) if movement too aggressive/sluggish

### Path Goes Through Obstacles

**Likely cause**: Grid resolution too coarse or obstacle inflation too small

**Fix**:
```python
# Finer grid resolution
GRID_RESOLUTION = 0.025  # was 0.05

# Or larger inflation
SAFETY_INFLATION_RADIUS_CELLS = 5  # was 3
```

### Planning Fails (No Path)

**Likely cause**: Start/goal in occupied space or surrounded by obstacles

**Fix**:
1. Check start/goal positions in `hw4_config.py`
2. Verify obstacle positions don't block path
3. Try `fast` mode (less conservative inflation):
   ```bash
   ros2 launch hw4_planning hw4_astar_planning.launch.py planner_mode:=fast
   ```

---

## Conclusion

HW4 achieves complete independence while leveraging HW2 concepts. The modular design allows:

✅ Separate testing of planning and control  
✅ Easy swapping of algorithm components  
✅ No namespace/dependency conflicts  
✅ Portable to other ROS2 workspaces  
✅ Clear separation of concerns  

