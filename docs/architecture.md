# 🏗️ System Architecture

## Overview

Beach Autonomous Systems uses a modular ROS 2 architecture for the ACC 2026 Self-Driving Car Competition.

## High-Level Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           SENSORS                                        │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐        │
│  │ Camera  │  │ Camera  │  │ LIDAR   │  │  IMU    │  │Encoders │        │
│  │ (Front) │  │ (Rear)  │  │ (360°)  │  │         │  │         │        │
│  └────┬────┘  └────┬────┘  └────┬────┘  └────┬────┘  └────┬────┘        │
└───────┼────────────┼───────────┼────────────┼────────────┼──────────────┘
        │            │           │            │            │
        └────────────┴───────────┴────────────┴────────────┘
                                 │
                    ┌────────────▼────────────┐
                    │      PERCEPTION         │
                    │  • Object Detection     │
                    │  • Lane Detection       │
                    │  • Sign Recognition     │
                    │  • Traffic Lights       │
                    └────────────┬────────────┘
                                 │
        ┌────────────────────────┼────────────────────────┐
        │                        │                        │
┌───────▼───────┐      ┌─────────▼─────────┐    ┌────────▼────────┐
│ LOCALIZATION  │      │     PLANNING      │    │    MAPPING      │
│ • Odometry    │      │ • Global Planner  │    │ (Future)        │
│ • Pose Est.   │      │ • Local Planner   │    │                 │
│ • TF          │      │ • Behavior FSM    │    │                 │
└───────┬───────┘      └─────────┬─────────┘    └─────────────────┘
        │                        │
        └────────────┬───────────┘
                     │
          ┌──────────▼──────────┐
          │      CONTROL        │
          │ • Path Following    │
          │ • PID Controllers   │
          │ • Speed Control     │
          └──────────┬──────────┘
                     │
          ┌──────────▼──────────┐
          │    VEHICLE (QCar 2) │
          │ • Steering Motor    │
          │ • Drive Motor       │
          └─────────────────────┘
```

## ROS 2 Node Graph

```
                            ┌──────────────────┐
                            │  /mission_node   │
                            │  (Ride Manager)  │
                            └────────┬─────────┘
                                     │
                    ┌────────────────┴────────────────┐
                    │                                 │
           ┌────────▼────────┐               ┌───────▼────────┐
           │/global_planner  │               │/behavior_node  │
           └────────┬────────┘               └───────┬────────┘
                    │                                 │
                    └────────────────┬────────────────┘
                                     │
                            ┌────────▼────────┐
                            │ /local_planner  │
                            └────────┬────────┘
                                     │
                            ┌────────▼────────┐
                            │ /controller     │
                            └────────┬────────┘
                                     │
                            ┌────────▼────────┐
                            │  /qcar_driver   │
                            └─────────────────┘
```

## Data Flow

### Perception Pipeline

```
Camera Image → Preprocessing → CNN Detection → Post-processing → Objects
     │
     └→ Lane Detection → Lane Lines
     │
     └→ Sign/Light Detection → Traffic Info
```

### Planning Pipeline

```
Goal + Map + Obstacles → Global Path → Local Trajectory → Control Commands
```

### Control Loop

```
While running:
    1. Get current pose from localization
    2. Get target trajectory from planning
    3. Compute control error
    4. Apply PID control
    5. Send commands to vehicle
    6. Loop at 50Hz
```

## Key Interfaces

### Topics

| Topic | Type | Publisher | Subscriber |
|-------|------|-----------|------------|
| `/camera/image` | sensor_msgs/Image | qcar_driver | perception |
| `/lidar/points` | sensor_msgs/PointCloud2 | qcar_driver | perception |
| `/perception/objects` | vision_msgs/DetectionArray | perception | planning |
| `/planning/path` | nav_msgs/Path | planning | control |
| `/cmd_vel` | geometry_msgs/Twist | control | qcar_driver |

### Services

| Service | Type | Node |
|---------|------|------|
| `/set_goal` | nav_msgs/srv/SetGoal | global_planner |
| `/emergency_stop` | std_srvs/srv/Trigger | controller |

## Technology Stack

| Layer | Technology |
|-------|------------|
| Middleware | ROS 2 Humble |
| Perception | OpenCV, PyTorch |
| Planning | Custom algorithms |
| Control | PID, Pure Pursuit |
| Simulation | Quanser QLabs |

---

*Beach Autonomous Systems - CSULB*
*Last Updated: January 2026*

