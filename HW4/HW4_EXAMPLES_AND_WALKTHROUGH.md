# HW4 Visual Examples & Walkthrough

## Example 1: A* Planning Visualization

### Occupancy Grid (Safety Mode)

```
World: 8ft × 8ft (2.44m × 2.44m)
Grid: 49×49 cells (5cm per cell)
Resolution: 5 cm/cell
Inflation: 3 cells (15 cm safety margin)

Legend: # = obstacle, . = free, S = start, G = goal, * = path

................................
................................
................................
................................
..............S...............
................................
................................
...#################...........
...#################...........
...#################...........
...#################...........
...#################...........
...#################...........
...#################...........
...#################...........
...#################...........
...#################...........
................................
................................
................................
............................G..
................................
................................
................................
................................
................................
................................
................................
```

### Planned Path (With A* Algorithm)

```
Path from (0.2m, 0.2m) to (2.2m, 2.2m):

Waypoint #0:  (0.200, 0.200), heading=50.0°
Waypoint #1:  (0.250, 0.250), heading=50.0°
Waypoint #2:  (0.300, 0.300), heading=50.0°
...
Waypoint #15: (1.100, 1.050), heading=45.0°   ← Approaching obstacle
Waypoint #16: (1.150, 1.100), heading=0.0°    ← Swerve around
Waypoint #17: (1.200, 1.250), heading=-45.0°  ← Swerve around
Waypoint #18: (1.250, 1.300), heading=-45.0°  ← Swerve around
...
Waypoint #30: (2.150, 2.150), heading=-45.0°
Waypoint #31: (2.200, 2.200), heading=0.0°    ← Goal reached

Total distance: 2.95m (vs 2.83m straight line)
Path ratio: 1.04× (very efficient!)
Planning time: ~50ms
```

---

## Example 2: Execution Log (CSV)

### Sample hw4_log.csv

```
t,state,x,y,yaw,gx,gy,gyaw,rho,eyaw,L,R
1.000,TRACKING,0.195,0.205,0.051,0.250,0.250,0.875,0.084,0.824,-0.02,0.15
1.050,TRACKING,0.220,0.230,0.701,0.250,0.250,0.875,0.032,0.174,-0.01,0.05
1.100,TRACKING,0.248,0.249,0.849,0.250,0.250,0.875,0.003,0.026,0.00,0.00
1.150,TRACKING,0.250,0.250,0.851,0.300,0.300,0.875,0.141,0.024,-0.00,0.09  ← Waypoint 1 reached
1.200,TRACKING,0.275,0.275,0.876,0.300,0.300,0.875,0.035,0.000,-0.00,0.01
1.250,TRACKING,0.298,0.299,0.876,0.300,0.300,0.875,0.002,0.000,0.00,0.00
1.300,TRACKING,0.300,0.300,0.875,0.350,0.350,0.875,0.141,0.000,-0.01,0.08  ← Waypoint 2 reached
...
5.200,TRACKING,1.090,1.050,0.780,1.150,1.100,1.789,0.084,1.009,-0.05,0.16  ← Approaching obstacle
5.250,TRACKING,1.100,1.075,0.720,1.150,1.100,1.789,0.067,-0.782,-0.04,0.12
5.300,TRACKING,1.120,1.095,0.500,1.200,1.250,1.789,0.191,-1.289,-0.12,0.08  ← Swerving around
5.350,TRACKING,1.145,1.120,0.250,1.200,1.250,1.789,0.150,-0.866,-0.08,0.05
5.400,TRACKING,1.170,1.140,0.100,1.200,1.250,1.789,0.138,-0.433,-0.09,0.01  ← Clear of obstacle
...
10.00,TRACKING,2.180,2.185,0.050,2.200,2.200,0.000,0.031,-0.050,0.00,0.02
10.05,TRACKING,2.198,2.199,0.000,2.200,2.200,0.000,0.003,0.000,0.00,0.00   ← Position tolerance reached
10.10,TRACKING,2.200,2.200,0.000,2.200,2.200,0.000,0.000,0.000,0.00,0.00   ← Heading aligned, done!
```

### State Transitions
```
Time 0.00-10.10:  TRACKING  (successfully following path, pose always fresh)
                  (if AprilTag lost at ~8 seconds):
Time 8.00:        TRACKING → COAST (pose becomes stale)
Time 8.20:        COAST → SEARCH (coast timeout reached)
Time 8.20-15.00:  SEARCH (rotating to regain pose)
Time 15.10:       SEARCH → TRACKING (pose reacquired, held for 0.15s)
Time 15.10+:      TRACKING (resume waypoint following)
```

---

## Example 3: Configuration for Different Scenarios

### Scenario A: Default (8ft × 8ft, Central Obstacle)
```python
# hw4_config.py
WORLD_MIN_X, WORLD_MAX_X = 0.0, 2.44
WORLD_MIN_Y, WORLD_MAX_Y = 0.0, 2.44

OBSTACLES = [(1.22, 1.22, 0.305, 0.305)]  # 2ft × 2ft in center

DEFAULT_START = (0.2, 0.2, 0.0)
DEFAULT_GOAL = (2.2, 2.2, 0.0)

GRID_RESOLUTION = 0.05
SAFETY_INFLATION_RADIUS_CELLS = 3
```

### Scenario B: Larger Space with Multiple Obstacles
```python
# hw4_config.py
WORLD_MIN_X, WORLD_MAX_X = 0.0, 4.0
WORLD_MIN_Y, WORLD_MAX_Y = 0.0, 4.0

OBSTACLES = [
    (1.5, 1.5, 0.3, 0.3),  # Central
    (0.5, 2.5, 0.2, 0.3),  # Left side
    (3.0, 2.5, 0.2, 0.3),  # Right side
    (2.0, 0.5, 0.3, 0.2),  # Bottom
]

DEFAULT_START = (0.3, 0.3, 0.0)
DEFAULT_GOAL = (3.7, 3.7, 0.0)

GRID_RESOLUTION = 0.05
SAFETY_INFLATION_RADIUS_CELLS = 4  # More conservative
```

### Scenario C: Fast Mode (Aggressive)
```python
# hw4_config.py
# Same world as Scenario A, but:

SAFETY_INFLATION_RADIUS_CELLS = 1  # Minimal safety margin
FAST_INFLATION_RADIUS_CELLS = 0    # No inflation in fast mode

MAX_V = 0.3  # Faster
MAX_W = 1.0
Kv = 0.8     # More aggressive control
Kw = 1.5
```

---

## Example 4: ROS Node Communication

### Startup Sequence (ros2 launch)

```
[INFO] [launch]: All log files can be found below /root/.ros/log/...
[INFO] [planning_node-1]: process started with pid [1234]
[INFO] [localization_hw4-2]: process started with pid [1235]
[INFO] [motor_controller_node-3]: process started with pid [1236]

[INFO] [planning_node-1]: ===== HW4 PLANNING NODE INITIALIZING =====
[INFO] [planning_node-1]: Planner mode: safety
[INFO] [planning_node-1]: Grid size: 49 x 49 cells
[INFO] [planning_node-1]: Planning from (0.2, 0.2) to (2.2, 2.2)
[INFO] [planning_node-1]: Path found with 31 waypoints
[INFO] [planning_node-1]: Waypoints with heading:
[INFO] [planning_node-1]:   0: (0.200, 0.200), yaw=50.0°
[INFO] [planning_node-1]:   1: (0.250, 0.250), yaw=50.0°
...
[INFO] [planning_node-1]:   30: (2.150, 2.150), yaw=-45.0°
[INFO] [planning_node-1]:   31: (2.200, 2.200), yaw=0.0°
[INFO] [planning_node-1]: ===== HW4 PLANNING NODE READY =====

[INFO] [waypoint_follower_hw4-1]: WaypointFollowerHW4 initialized with 31 waypoints
[INFO] [localization_hw4-2]: HW4 LocalizationNode initialized

[INFO] [waypoint_follower_hw4-1]: Reached waypoint 0: (0.20,0.20,50.00)
[INFO] [waypoint_follower_hw4-1]: Reached waypoint 1: (0.25,0.25,50.00)
[INFO] [waypoint_follower_hw4-1]: Reached waypoint 2: (0.30,0.30,50.00)
...
[INFO] [waypoint_follower_hw4-1]: Reached waypoint 30: (2.15,2.15,-45.00)
[INFO] [waypoint_follower_hw4-1]: Reached waypoint 31: (2.20,2.20,0.00)
```

### Topic Publishing

```bash
$ ros2 topic list
/motor_commands
/pose_estimated
/parameter_events
/rosout

$ ros2 topic echo /motor_commands
data:
- -0.05
- 0.15
---
data:
- -0.02
- 0.12
---

$ ros2 topic echo /pose_estimated
header:
  stamp:
    sec: 1234567890
    nsec: 123456000
  frame_id: world
pose:
  position:
    x: 1.05
    y: 1.15
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.259  # sin(heading/2)
    w: 0.966  # cos(heading/2)
---
```

---

## Example 5: Troubleshooting Output

### Problem: No Path Found

```
[ERROR] [planning_node-1]: No path found!
Traceback (most recent call last):
  ...
  RuntimeError: A* planning failed - no path exists

DIAGNOSIS:
1. Check if goal is in free space
2. Check obstacle positions
3. Try with --planner_mode=fast (less inflation)
```

**Solution**: Adjust DEFAULT_GOAL in config to be further from obstacles

### Problem: Robot Moves Too Slowly

```
CSV Log shows small motor commands:
t,state,x,y,yaw,gx,gy,gyaw,rho,eyaw,L,R
10.00,TRACKING,0.5,0.5,0.0,1.0,1.0,0.0,0.707,0.0,-0.01,0.02  ← Small L,R

DIAGNOSIS:
- Kv too small (distance gain)
- MAX_V limited
- Robot wheels slipping

SOLUTION:
Kv = 0.6  →  Kv = 1.0     (increase gain)
MAX_V = 0.2  →  MAX_V = 0.3  (increase limit)
```

### Problem: Robot Oscillates Around Waypoint

```
CSV Log shows erratic heading:
t,state,x,y,yaw,gx,gy,gyaw,rho,eyaw,L,R
5.00,TRACKING,1.0,1.0,0.2,1.1,1.1,0.0,0.141,0.2,-0.05,0.08
5.05,TRACKING,1.05,1.04,0.5,1.1,1.1,0.0,0.092,0.5,-0.15,0.18
5.10,TRACKING,1.08,1.09,0.0,1.1,1.1,0.0,0.031,-0.3,0.09,-0.02  ← Oscillating yaw

DIAGNOSIS:
- Kw too large (heading gain)
- Robot correcting too aggressively

SOLUTION:
Kw = 1.2  →  Kw = 0.8     (decrease gain)
```

---

## Example 6: Parameter Tuning Guide

### Initial Tuning Process

```
1. START: Default parameters
   Kv = 0.6, Kw = 1.2, MAX_V = 0.2, MAX_W = 0.8

2. BEHAVIOR OBSERVATION
   ┌─────────────────────┬──────────────────────┐
   │ Observation         │ Adjustment           │
   ├─────────────────────┼──────────────────────┤
   │ Moves slowly        │ Kv up, MAX_V up      │
   │ Can't turn          │ Kw up, MAX_W up      │
   │ Oscillates yaw      │ Kw down              │
   │ Overshoots waypoint │ Kv down              │
   │ Hits obstacles      │ Increase inflation   │
   │ Path too long       │ Decrease inflation   │
   └─────────────────────┴──────────────────────┘

3. TUNING ITERATIONS
   Iteration 1: Kv=0.8 (was slow)
               → Still slow
   Iteration 2: Kv=1.0, MAX_V=0.25
               → Better, slight oscillation
   Iteration 3: Kv=0.9, Kw=1.0
               → Good!

4. FINAL PARAMETERS
   Kv = 0.9
   Kw = 1.0
   MAX_V = 0.25
   MAX_W = 0.85
```

---

## Example 7: Comparing Planner Modes

### Safety Mode vs Fast Mode

```
Command: ros2 launch hw4_planning hw4_astar_planning.launch.py --planner_mode=safety

Safety Mode:
├─ Inflation: 3 cells (15 cm margin)
├─ Path waypoints: 31
├─ Total distance: 2.95 m
├─ Time to plan: 45 ms
├─ Clearance from obstacles: ~20 cm
├─ Risk: Very low (conservative)
└─ Use when: Near valuable obstacles, high safety priority

vs

Fast Mode:
├─ Inflation: 1 cell (5 cm margin)
├─ Path waypoints: 38
├─ Total distance: 2.78 m
├─ Time to plan: 28 ms
├─ Clearance from obstacles: ~5 cm
├─ Risk: Higher (tight paths)
└─ Use when: Open space, performance priority

Path comparison (ASCII):
Safety:  ⊗ ⊗ ⊗ ⊗ ⊗ ⊗ ⊗ ⊗ ⊗ ⊗ (clear of center obstacle)
                ||||||||||| (wide swerve around)

Fast:    ⊗ ⊗ ⊗ ⊗ ⊗ ⊗ ⊗ ⊗ ⊗ ⊗ (close to obstacle)
             ||||||||| (tighter curve)
```

---

## Summary: System Behavior at a Glance

```
                    Start
                      |
                      ▼
            ┌─────────────────────┐
            │  Plan A* Path       │  
            │  (31 waypoints)     │  Time: ~50ms
            │  Avoid obstacles    │
            └──────────┬──────────┘
                       │
                       ▼
         ┌─────────────────────────────┐
         │ Launch Waypoint Follower    │
         │ Subscribe: /pose_estimated  │
         │ Publish: /motor_commands    │
         └──────────┬──────────────────┘
                    │
                    ▼
         ┌──────────────────────────────┐
         │ Enter TRACKING State         │
         │ Wait for pose estimate       │
         └──────────┬───────────────────┘
                    │ (pose arrives)
                    ▼
    ┌────────────────────────────────────────┐
    │ TRACKING Loop (20 Hz)                  │
    │ • Read current pose (x, y, yaw)        │
    │ • Compute distance to goal             │
    │ • Apply P-gain control                 │
    │ • Send motor commands                  │
    │ • Log state to CSV                     │
    │                                        │
    │ Distance < threshold?                  │
    │  ├─ NO:  Continue tracking             │
    │  └─ YES: Move to next waypoint         │
    └────────────────────────────────────────┘
                    │
                    ├─ (pose lost)───────────┐
                    │                        │
                    ▼                        ▼
              TRACKING ────────────► COAST ─→ SEARCH
                    ▲                          │
                    └──────────────────────────┘
                       (pose reacquired)
                    │
                    ├─ All waypoints reached?
                    │  ├─ NO:  Continue to next waypoint
                    │  └─ YES: Final heading alignment
                    │
                    ▼
              DONE - Robot stopped
         (Log saved: ~/hw4_log.csv)
```

This visual system demonstrates the complete flow from planning to execution!
