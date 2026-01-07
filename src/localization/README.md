# 📍 Localization Package

This package handles vehicle localization and mapping.

## Responsibilities

- Position estimation
- Odometry computation
- Map management
- Coordinate transformations

## Nodes

| Node | Description | Topics |
|------|-------------|--------|
| `odometry_node` | Computes wheel odometry | `/wheel/encoders` → `/odom` |
| `localization_node` | Estimates global position | `/odom`, `/gps` → `/pose` |
| `tf_broadcaster` | Publishes transforms | Various → `/tf` |

## Coordinate Frames

```
              map
               │
               ▼
             odom
               │
               ▼
           base_link
          ┌────┼────┐
          ▼    ▼    ▼
       front  lidar  camera
       wheel
```

## Transform Tree

| Parent | Child | Description |
|--------|-------|-------------|
| `map` | `odom` | Map to odometry (localization correction) |
| `odom` | `base_link` | Odometry to vehicle base |
| `base_link` | `lidar_link` | Base to LIDAR sensor |
| `base_link` | `camera_link` | Base to camera sensor |

## Localization Methods

1. **Wheel Odometry**
   - Uses encoder data
   - Accumulates drift over time
   - Good for short-term

2. **GPS Fusion** 
   - Corrects odometry drift
   - May have noise/latency

3. **Visual Odometry** 
   - Uses camera features
   - Complements wheel odometry

## Usage

```bash
# Run localization nodes
ros2 launch localization localization_launch.py

# View TF tree
ros2 run tf2_tools view_frames
```

---

*Beach Autonomous Systems - CSULB*

