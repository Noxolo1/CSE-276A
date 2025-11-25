# HW4 Quick Reference

## Files Overview

```
hw4_planning/
├── hw4_planning/
│   ├── __init__.py                    # Package marker
│   ├── astar_planner.py              # A* algorithm (core planning)
│   ├── hw4_config.py                 # Configuration (world, obstacles, gains)
│   ├── planning_node.py              # Main orchestrator (A* + launcher)
│   ├── waypoint_follower_hw4.py      # Robot control (from HW2 concepts)
│   ├── localization_hw4.py           # Pose estimation (AprilTags)
│   └── motor_control.py              # Motor interface (optional)
├── launch/
│   └── hw4_astar_planning.launch.py  # Launch file
├── configs/
│   └── apriltags_position.yaml       # Tag positions
├── test_planner.py                   # Standalone test script
└── README.md                         # Full documentation
```

---

## Quick Start

### 1. Install Dependencies
```bash
sudo apt install python3-scipy python3-numpy ros2-tf-transformations
```

### 2. Build Package
```bash
cd ~/colcon_ws
colcon build --packages-select hw4_planning
source install/setup.bash
```

### 3. Test Planner (No ROS)
```bash
cd HW4/hw4_planning
python3 test_planner.py
```

### 4. Run Full System
```bash
# Safety mode (recommended)
ros2 launch hw4_planning hw4_astar_planning.launch.py

# Fast mode (less conservative)
ros2 launch hw4_planning hw4_astar_planning.launch.py planner_mode:=fast
```

---

## Configuration Checklist

Before running, verify in `hw4_config.py`:

- [ ] World bounds match your workspace
- [ ] Obstacle position and size correct
- [ ] AprilTag positions in `MAP_TAGS` match physical tags
- [ ] Start/goal positions are in free space
- [ ] Control gains (Kv, Kw) appropriate for robot
- [ ] Motor velocity limits (MAX_V, MAX_W) safe

---

## Key Parameters

### World Configuration
```python
WORLD_MIN_X, WORLD_MAX_X = 0.0, 2.44      # Workspace X (meters)
WORLD_MIN_Y, WORLD_MAX_Y = 0.0, 2.44      # Workspace Y (meters)
GRID_RESOLUTION = 0.05                     # Cell size (meters)
```

### Obstacles
```python
OBSTACLES = [
    (center_x, center_y, half_width, half_height),
    (1.22, 1.22, 0.305, 0.305),  # Default: 2ft×2ft in center
]
```

### Planning Modes
```python
# Safety mode: 3-cell inflation (~15 cm margin)
SAFETY_INFLATION_RADIUS_CELLS = 3

# Fast mode: 1-cell inflation (~5 cm margin)
FAST_INFLATION_RADIUS_CELLS = 1
```

### Robot Control (from HW2)
```python
MAX_V = 0.2          # m/s (max linear velocity)
MAX_W = 0.80         # rad/s (max angular velocity)
Kv = 0.6             # Distance gain (lower = slower response)
Kw = 1.2             # Heading gain (higher = tighter turns)
BASELINE = 0.127     # m (wheel track width)
```

### Tolerances
```python
POS_TOL = 0.12       # m (position tolerance)
YAW_TOL = 10°        # rad (heading tolerance)
```

---

## Launch Modes

### Standard (Safety)
```bash
ros2 launch hw4_planning hw4_astar_planning.launch.py
```
- Maximum obstacle inflation
- Safest paths
- Slower planning preferred

### Fast Mode
```bash
ros2 launch hw4_planning hw4_astar_planning.launch.py planner_mode:=fast
```
- Minimum inflation
- Shorter paths
- Riskier (closer to obstacles)

### Debug Mode (Verbose Logging)
```bash
ros2 launch hw4_planning hw4_astar_planning.launch.py 2>&1 | tee hw4.log
```

---

## ROS Topics

### Subscribed Topics
| Topic | Type | Source |
|-------|------|--------|
| `/pose_estimated` | `PoseStamped` | Localization node |
| `/apriltag_pose` | `PoseStamped` | AprilTag detector |

### Published Topics
| Topic | Type | Publisher |
|-------|------|-----------|
| `/motor_commands` | `Float32MultiArray` | Waypoint follower |
| `/pose_estimated` | `PoseStamped` | Localization node |

### Topic Formats

**Motor Commands** (Float32MultiArray):
```python
msg.data = [left_velocity, right_velocity]  # m/s
```

**Pose Estimated** (PoseStamped):
```python
msg.header.frame_id = "world"
msg.pose.position = (x, y, z)              # meters
msg.pose.orientation = (x, y, z, w)        # quaternion
```

---

## Output Files

### Occupancy Grid
**File**: `~/occupancy_grid.txt`
```
# # = occupied, . = free
................................
#############################...
#############################...
```

### Execution Log
**File**: `~/hw4_log.csv`
```
time,state,x,y,yaw,gx,gy,gyaw,rho,eyaw,L,R
1.234,TRACKING,0.1,0.1,0.0,1.0,1.0,0.0,0.9,0.1,-0.05,0.15
```

---

## State Machine Diagram

```
         TRACKING
         /      \
      pose?    !pose?
      /          \
     Y            N
     |            |
     |   COAST ──┐
     |    |  \   |
     |    |   └──┴─→ SEARCH
     |    |          /  |
     └────┤◄─────────   |
          |          (timeout)
          |            |
          └────────────┴──→ FAILSAFE
                               |
                           (pose?)
                               |
                              TRACKING
```

---

## Common Issues & Fixes

| Issue | Cause | Fix |
|-------|-------|-----|
| "No path found" | Start/goal in obstacles | Adjust start/goal in config |
| Robot slow | Gains too low | Increase Kv, Kw |
| Robot overshoots | Gains too high | Decrease Kv, Kw |
| Path goes through obstacles | Inflation too small | Increase inflation radius |
| Planner hangs | Grid too fine | Increase `GRID_RESOLUTION` |
| Motor commands not received | Topic name wrong | Check `/motor_commands` topic |
| Pose never updates | AprilTags not visible | Check tag visibility |

---

## Performance Tips

### For Faster Planning
```python
# Coarser grid
GRID_RESOLUTION = 0.1  # was 0.05

# Less inflation
SAFETY_INFLATION_RADIUS_CELLS = 1  # was 3
```

### For Smoother Paths
```python
# Finer grid
GRID_RESOLUTION = 0.025  # was 0.05

# More inflation
SAFETY_INFLATION_RADIUS_CELLS = 5  # was 3
```

### For Faster Robot Movement
```python
# Increase velocity limits (careful!)
MAX_V = 0.3   # was 0.2
MAX_W = 1.0   # was 0.80

# Increase gains
Kv = 1.0      # was 0.6
Kw = 2.0      # was 1.2
```

---

## Testing Commands

### Test without ROS
```bash
cd HW4/hw4_planning
python3 test_planner.py --mode safety --show-grid --show-path
```

### Check topics
```bash
ros2 topic list                           # All topics
ros2 topic echo /pose_estimated          # Pose messages
ros2 topic echo /motor_commands          # Motor commands
```

### Monitor node
```bash
ros2 node list                            # All running nodes
ros2 node info /hw4_planning              # Node details
```

### View logs
```bash
cat ~/hw4_log.csv | head -20              # First 20 lines
tail -f ~/hw4_log.csv                     # Live update
```

---

## Debugging Tips

1. **Enable verbose logging**:
   ```python
   self.get_logger().info(f"Debug message: {variable}")
   ```

2. **Save grid for inspection**:
   ```python
   node = Hw4PlanningNode()
   node.log_grid("debug_grid.txt")
   ```

3. **Check A* directly**:
   ```bash
   python3 test_planner.py --no-show-grid --show-path
   ```

4. **Monitor state machine**:
   Look for state transitions in CSV log: TRACKING→COAST→SEARCH→...

---

## Summary

| Aspect | What | Where |
|--------|------|-------|
| **Planning** | A* grid search | `astar_planner.py` |
| **Configuration** | World/obstacles | `hw4_config.py` |
| **Control** | Waypoint following | `waypoint_follower_hw4.py` |
| **Localization** | Pose estimation | `localization_hw4.py` |
| **Integration** | Orchestration | `planning_node.py` |
| **Launch** | System startup | `hw4_astar_planning.launch.py` |

---

## Need Help?

- Full documentation: `HW4_ASTAR_README.md`
- Integration guide: `HW4_INTEGRATION_GUIDE.md`
- Test script: `test_planner.py --help`
- Node logs: `ros2 launch ... 2>&1 | tee debug.log`
