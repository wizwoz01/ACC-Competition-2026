# 📦 ROS 2 Source Packages

This directory contains the ROS 2 packages for the Beach Autonomous Systems self-driving car.

## Package Overview

```
src/
├── perception/      # Sensor processing and object detection
├── planning/        # Path planning and decision making
├── control/         # Vehicle control
├── localization/    # Position estimation
└── bringup/         # Launch files and configuration
```

## Package Descriptions

### [Perception](perception/)
Handles all sensor data processing:
- Camera image processing
- LIDAR point cloud processing
- Object detection
- Traffic light/sign recognition
- Lane detection

### [Planning](planning/)
Handles decision making and path generation:
- Global path planning
- Local trajectory planning
- Behavior planning
- Mission management

### [Control](control/)
Handles vehicle actuation:
- Steering control
- Speed control
- Path following algorithms

### [Localization](localization/)
Handles position estimation:
- Odometry computation
- Sensor fusion
- Coordinate transforms

## Building

```bash
# Navigate to workspace root
cd /workspace

# Build all packages
colcon build

# Build specific package
colcon build --packages-select perception

# Source the workspace
source install/setup.bash
```

## Running

```bash
# Launch full system
ros2 launch bringup full_system.launch.py

# Launch individual subsystem
ros2 launch perception perception.launch.py
ros2 launch planning planning.launch.py
ros2 launch control control.launch.py
ros2 launch localization localization.launch.py
```

## Topic Map

```
                    ┌─────────────────┐
                    │    Sensors      │
                    │ (Camera, LIDAR) │
                    └────────┬────────┘
                             │
                    ┌────────▼────────┐
                    │   Perception    │
                    │  /perception/*  │
                    └────────┬────────┘
                             │
              ┌──────────────┼──────────────┐
              │              │              │
     ┌────────▼────────┐     │     ┌────────▼────────┐
     │  Localization   │     │     │    Planning     │
     │     /odom       │     │     │  /planning/*    │
     └────────┬────────┘     │     └────────┬────────┘
              │              │              │
              └──────────────┴──────────────┘
                             │
                    ┌────────▼────────┐
                    │    Control      │
                    │    /cmd_vel     │
                    └────────┬────────┘
                             │
                    ┌────────▼────────┐
                    │   QCar 2 HW     │
                    └─────────────────┘
```

## Development

See [Development Guide](../Software/Development_Guide.md) for workflow details.

---

*Beach Autonomous Systems - CSULB*

