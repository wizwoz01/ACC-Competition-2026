# 💻 Software Development Guide - ACC 2026

This directory contains development guides and utilities for the ACC 2026 Self-Driving Car Competition.

## Contents

- [Development Guide](Development_Guide.md) - Main development workflow
- [ROS2 Setup](ROS2_Setup.md) - ROS 2 Humble configuration
- [QLabs Guide](QLabs_Guide.md) - Quanser Interactive Labs usage

## Development Stack

| Component | Technology |
|-----------|------------|
| OS | Ubuntu 24.04 |
| Middleware | ROS 2 Humble |
| Simulation | Quanser Interactive Labs (QLabs) |
| Language | Python 3.10+ |
| Container | Docker |

## Quick Reference

### ROS 2 Commands
```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Build workspace
colcon build

# Source workspace
source install/setup.bash

# Run a node
ros2 run <package_name> <node_name>

# Launch a system
ros2 launch <package_name> <launch_file>
```

### QLabs Commands
```bash
# Launch QLabs (from container)
# Specific commands depend on setup - see ROS Technical Resources
```

## Recommended Development Workflow

1. **Setup Environment**
   - Start Docker container
   - Source ROS 2 and workspace

2. **Develop**
   - Write code in `src/` packages
   - Use VS Code with Remote Containers extension

3. **Build & Test**
   - Build with `colcon build`
   - Test in QLabs simulation

4. **Iterate**
   - Debug using ROS 2 tools
   - Refine algorithms

## ROS 2 Package Structure

```
src/
├── perception/           # Sensor processing
│   ├── camera_node.py
│   ├── lidar_node.py
│   └── detection_node.py
├── planning/             # Decision making
│   ├── path_planner.py
│   └── behavior_tree.py
├── control/              # Vehicle control
│   ├── controller.py
│   └── pid_controller.py
└── localization/         # Position estimation
    ├── odometry.py
    └── mapping.py
```

## Useful Tools

- **RViz2** - Visualization
- **rqt** - GUI tools
- **rosbag2** - Data recording
- **Plotjuggler** - Data plotting

## Resources

- [ROS 2 Humble Docs](https://docs.ros.org/en/humble/)
- [ROS Technical Resources](https://github.com/quanser/student-competition-resources-ros)
- [QLabs Documentation](https://docs.quanser.com/qlabs/)

---

*Beach Autonomous Systems - CSULB*

