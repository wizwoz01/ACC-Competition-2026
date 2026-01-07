# 📷 Perception Package

This package handles sensor data processing for the QCar 2.

## Responsibilities

- Camera image processing
- LIDAR point cloud processing
- Object detection (vehicles, pedestrians, signs)
- Traffic light recognition
- Lane detection

## Nodes

| Node | Description | Topics |
|------|-------------|--------|
| `camera_processor` | Processes camera images | `/camera/image_raw` → `/camera/processed` |
| `lidar_processor` | Processes LIDAR data | `/lidar/points` → `/lidar/filtered` |
| `object_detector` | Detects objects | `/camera/processed` → `/perception/objects` |
| `traffic_light_detector` | Detects traffic lights | `/camera/processed` → `/perception/traffic_lights` |
| `lane_detector` | Detects lane markings | `/camera/processed` → `/perception/lanes` |

## Dependencies

- OpenCV
- NumPy
- sensor_msgs
- cv_bridge

## Usage

```bash
# Run perception nodes
ros2 launch perception perception_launch.py
```

## Development Notes

- Input images are in BGR format
- LIDAR returns point cloud in vehicle frame
- Detection confidence threshold: configurable via parameters

---

*Beach Autonomous Systems - CSULB*

