# HW4 Documentation Index

Welcome to the HW4 A* Path Planning & Robot Navigation solution!

This document serves as an **index and roadmap** through all the documentation and code.

---

## 📋 Quick Navigation

### For First-Time Users
1. **Start here**: [HW4_QUICK_REFERENCE.md](HW4_QUICK_REFERENCE.md)
   - 5-minute overview
   - Installation steps
   - Running the system
   - Basic troubleshooting

2. **Then read**: [HW4_IMPLEMENTATION_SUMMARY.md](HW4_IMPLEMENTATION_SUMMARY.md)
   - What was created
   - File organization
   - Design decisions
   - Key algorithms

3. **See examples**: [HW4_EXAMPLES_AND_WALKTHROUGH.md](HW4_EXAMPLES_AND_WALKTHROUGH.md)
   - Visual examples of planning
   - CSV output samples
   - State machine behavior
   - Tuning guide

### For Detailed Understanding
4. **Deep dive**: [HW4_ASTAR_README.md](HW4_ASTAR_README.md)
   - Complete algorithm description
   - Architecture documentation
   - Building and running
   - Configuration reference
   - Output formats
   - Comprehensive troubleshooting

5. **Integration info**: [HW4_INTEGRATION_GUIDE.md](HW4_INTEGRATION_GUIDE.md)
   - Design philosophy
   - How it relates to HW2
   - Integration points
   - Extending the system
   - Testing strategies

---

## 📁 Code Structure

```
hw4_planning/
│
├─ hw4_planning/                    Main package code
│  ├─ astar_planner.py              A* algorithm
│  ├─ hw4_config.py                 Configuration (edit this!)
│  ├─ planning_node.py              Main orchestrator
│  ├─ waypoint_follower_hw4.py      Robot control
│  └─ localization_hw4.py           Pose estimation
│
├─ launch/
│  └─ hw4_astar_planning.launch.py  ROS2 launch file
│
├─ test_planner.py                  Standalone test (no ROS needed)
│
└─ package.xml, setup.py            Build configuration
```

### Code Documentation Summary

| File | Purpose | Lines | Key Class |
|------|---------|-------|-----------|
| `astar_planner.py` | A* grid search algorithm | ~300 | `AStarPlanner` |
| `hw4_config.py` | Central configuration | ~80 | (constants) |
| `planning_node.py` | Orchestrator | ~200 | `Hw4PlanningNode` |
| `waypoint_follower_hw4.py` | Robot control | ~350 | `WaypointFollowerHW4` |
| `localization_hw4.py` | Pose estimation | ~150 | `LocalizationNode` |
| `test_planner.py` | Offline testing | ~300 | (functions) |

---

## 🚀 Getting Started

### Installation (< 5 minutes)

```bash
# 1. Install dependencies
sudo apt install python3-scipy python3-numpy ros2-tf-transformations

# 2. Build package
cd ~/colcon_ws
colcon build --packages-select hw4_planning
source install/setup.bash

# 3. Test (no ROS)
cd HW4/hw4_planning
python3 test_planner.py

# 4. Run (with ROS)
ros2 launch hw4_planning hw4_astar_planning.launch.py
```

### First Commands

```bash
# View quick reference
cat HW4_QUICK_REFERENCE.md

# Test A* offline
python3 test_planner.py --mode safety --show-grid

# Run full system
ros2 launch hw4_planning hw4_astar_planning.launch.py

# Monitor topics
ros2 topic echo /motor_commands
ros2 topic echo /pose_estimated

# Check logs
cat ~/hw4_log.csv | head -20
cat ~/occupancy_grid.txt
```

---

## 📚 Documentation Guide

### Overview Documents (30 minutes)

| Document | Time | Focus | Best For |
|----------|------|-------|----------|
| Quick Reference | 5 min | Essentials | Getting started |
| Implementation Summary | 15 min | What was created | Understanding structure |
| Examples & Walkthrough | 10 min | Visual examples | Seeing it in action |

### Detailed Documents (45 minutes)

| Document | Time | Focus | Best For |
|----------|------|-------|----------|
| A* README | 25 min | Complete details | Full understanding |
| Integration Guide | 20 min | How it works | Extending/customizing |

**Total reading time**: ~75 minutes for comprehensive understanding

---

## 🔍 How to Use This Documentation

### "How do I..."

| Question | Answer Location |
|----------|-----------------|
| Install and run the system? | Quick Reference → Getting Started |
| Understand the architecture? | Implementation Summary → Architecture |
| See what's inside each file? | Implementation Summary → New Components |
| Modify configuration? | Quick Reference → Key Parameters |
| Tune control gains? | Examples → Parameter Tuning Guide |
| Understand the algorithms? | A* README → Algorithm Details |
| Extend with new features? | Integration Guide → How to Extend |
| Debug problems? | A* README → Troubleshooting |
| Test offline? | Quick Reference → Testing Commands |
| Understand state machine? | Examples → State Machine Behavior |

---

## 📊 Key Concepts at a Glance

### A* Path Planning

**Grid-based pathfinding** with obstacle inflation:
- Converts world coordinates to grid cells (5 cm each)
- Builds occupancy grid with inflated obstacles
- Searches using A* algorithm with Euclidean heuristic
- Returns optimal path avoiding obstacles

**Two modes**:
- Safety: 3-cell inflation (15 cm margin) → conservative paths
- Fast: 1-cell inflation (5 cm margin) → shorter paths

### Robot Control

**Proportional guidance law**:
- Distance to goal → linear velocity: $v = K_v \rho$
- Heading error → angular velocity: $\omega = K_w e_{yaw}$
- Converts to differential drive motor commands

**State machine**:
1. TRACKING: Normal waypoint following (fresh pose)
2. COAST: Brief stop before searching
3. SEARCH: Rotate to regain visual lock
4. FAILSAFE: Stopped, waiting for pose

### Integration with HW2

**Reuses concepts** (proportional guidance, state machine):
- Independent implementation (no code imports)
- Same robot model (differential drive)
- Same message types (motor_commands, pose_estimated)
- Cleaner separation than importing HW2

---

## 🎯 Common Tasks

### Task: Run A* Planning
```bash
ros2 launch hw4_planning hw4_astar_planning.launch.py --planner_mode=safety
```
→ See HW4_QUICK_REFERENCE.md → Launch Modes

### Task: Change World Bounds
1. Edit `hw4_config.py`
2. Modify `WORLD_MIN_X`, `WORLD_MAX_X`, etc.
3. Rebuild and rerun
→ See HW4_ASTAR_README.md → Configuration

### Task: Add New Obstacle
1. Edit `hw4_config.py` → `OBSTACLES` list
2. Add tuple: `(center_x, center_y, half_width, half_height)`
3. Rerun planner
→ See HW4_INTEGRATION_GUIDE.md → How to Extend

### Task: Tune Robot Control
1. Monitor `/motor_commands` topic
2. Observe robot behavior
3. Adjust `Kv`, `Kw`, `MAX_V`, `MAX_W` in config
4. Iterate
→ See HW4_EXAMPLES_AND_WALKTHROUGH.md → Parameter Tuning

### Task: Debug Planning Failure
1. Check `DEFAULT_START` and `DEFAULT_GOAL` in config
2. Verify obstacle positions
3. Run test: `python3 test_planner.py`
4. Try `--planner_mode=fast` for less inflation
→ See HW4_ASTAR_README.md → Troubleshooting

---

## 🔧 File Reference

### Configuration Files

| File | Purpose | How to Edit |
|------|---------|-----------|
| `hw4_config.py` | World, obstacles, control gains | Direct editing |
| `package.xml` | Dependencies | Colcon build auto-updates |
| `setup.py` | Entry points | Auto-generated |

### Output Files

| File | Purpose | Location |
|------|---------|----------|
| `hw4_log.csv` | Execution log (state, pose, commands) | `~/hw4_log.csv` |
| `occupancy_grid.txt` | ASCII grid visualization | `~/occupancy_grid.txt` |

### Testing Files

| File | Purpose | Run |
|------|---------|-----|
| `test_planner.py` | Offline A* test | `python3 test_planner.py` |

---

## 🧪 Testing & Validation

### Level 1: Unit Testing (No ROS)
```bash
python3 test_planner.py --mode safety --show-grid --show-path
```
**Tests**: A* algorithm, grid building, path validation

### Level 2: Integration Testing (With ROS)
```bash
ros2 launch hw4_planning hw4_astar_planning.launch.py
```
**Tests**: Full system, pose estimation, control loop

### Level 3: Manual Testing (Physical Robot)
1. Verify obstacle positions match configuration
2. Ensure AprilTags visible and positions correct
3. Start system and monitor `/pose_estimated` topic
4. Observe robot movement vs planned path
5. Check `/motor_commands` for expected values

→ See HW4_ASTAR_README.md → Testing & Validation

---

## 📈 Performance Metrics

### Planning Performance
- Grid size: ~2500 cells (49×49)
- Planning time: ~40-50 ms
- Path length: 2.83-2.95 m (efficiency: 1.04×)
- Memory: ~50 KB (occupancy grid)

### Robot Performance
- Control loop: 20 Hz (50 ms)
- Waypoint tolerance: 12 cm
- Heading tolerance: 10°
- Max velocity: 0.2 m/s
- Max angular velocity: 0.8 rad/s

→ See HW4_ASTAR_README.md → Architecture

---

## ❓ FAQ

**Q: Can I run this without HW2?**
A: Yes! HW4 is completely independent. See HW4_INTEGRATION_GUIDE.md → Self-Contained.

**Q: Do I need to modify the code?**
A: Usually only `hw4_config.py`. Code is generic enough for most setups.

**Q: Can I test without a robot?**
A: Yes! Use `test_planner.py` or simulate pose messages.

**Q: How do I change planning algorithm?**
A: See HW4_INTEGRATION_GUIDE.md → How to Extend → Swap Planners.

**Q: What if A* finds no path?**
A: See HW4_ASTAR_README.md → Troubleshooting → "No Path Found".

---

## 📞 Debugging Help

### Check Planner
```bash
python3 test_planner.py --mode safety --show-path
# Look for: path found, waypoints printed, no obstacles in path
```

### Check Localization
```bash
ros2 topic echo /pose_estimated
# Look for: position (x,y), orientation (quaternion), regular updates
```

### Check Control
```bash
ros2 topic echo /motor_commands
# Look for: left/right motor values, non-zero when moving
```

### Check Logs
```bash
tail -f ~/hw4_log.csv
# Look for: state transitions, position progressing toward goal
```

→ See HW4_ASTAR_README.md → Troubleshooting

---

## 🎓 Learning Path

### Beginner
1. Quick Reference (overview)
2. Examples & Walkthrough (see it in action)
3. Run `test_planner.py` (test offline)
4. Run `ros2 launch` (test with ROS)

### Intermediate
1. Implementation Summary (architecture)
2. A* README (detailed docs)
3. Modify `hw4_config.py` (change parameters)
4. Monitor ROS topics (debug)

### Advanced
1. Integration Guide (design philosophy)
2. Read source code (`*.py` files)
3. Implement extensions (new planners, controllers)
4. Customize for your robot

---

## 📝 Document Cheat Sheet

| Document | When to Read | What You'll Learn |
|----------|--------------|-------------------|
| Quick Reference | First 5 min | How to get running |
| Implementation Summary | Next 15 min | What was built |
| Examples & Walkthrough | Next 10 min | Visual examples |
| A* README | For details | Everything |
| Integration Guide | To extend | How to customize |

---

## ✅ Success Indicators

### ✓ System is working if:
- `test_planner.py` shows path without errors
- `ros2 launch` starts all nodes successfully
- `/motor_commands` topic publishing
- `/pose_estimated` topic publishing
- `~/hw4_log.csv` contains state machine trace
- Robot moves toward first waypoint

### ✓ Tuning is good if:
- Robot follows path smoothly
- No oscillation around waypoints
- Avoids obstacles with margin
- State machine: mostly TRACKING, brief COAST/SEARCH
- Motors respond proportionally to distance

---

## 🚀 Next Steps

1. **Read**: Start with [HW4_QUICK_REFERENCE.md](HW4_QUICK_REFERENCE.md)
2. **Install**: Follow installation steps
3. **Test**: Run `python3 test_planner.py`
4. **Configure**: Edit `hw4_config.py` for your setup
5. **Run**: Launch the system
6. **Monitor**: Check topics and logs
7. **Extend**: Implement your own features!

---

**Happy planning! 🤖**

For specific questions, check the "How do I..." table above or use Ctrl+F to search these documents.
