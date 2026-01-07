# 🟦 MATLAB Development

This is where the self-driving algorithm for the ACC 2026 competition is developed.

---

## 📁 Folder Structure

```
matlab/
├── models/           # Simulink models
├── scripts/          # MATLAB scripts
├── functions/        # Reusable functions
└── README.md         # This file
```

---

## 🚀 Quick Start

### 1. Launch QLabs
```matlab
QLabs.launch
```

### 2. Navigate to Open Plane in QLabs GUI

### 3. Spawn the QCar
```matlab
qcar = QCar2();
qcar.spawn([0, 0, 0], 0);  % [x, y, z], heading
```

### 4. Run the Main Script
```matlab
run('scripts/main.m')
```

### 5. Clean Up When Done
```matlab
qcar.terminate();
```

---

## 📂 What Goes Where

### `models/` - Simulink Models
- `vehicle_control.slx` - Main vehicle controller
- `path_planning.slx` - Path planning system
- `perception.slx` - Sensor processing pipeline

### `scripts/` - MATLAB Scripts
- `main.m` - Main entry point
- `setup_qcar.m` - QCar initialization
- `test_*.m` - Test scripts

### `functions/` - Reusable Functions
- `pid_controller.m` - PID control function
- `pure_pursuit.m` - Path following algorithm
- `process_lidar.m` - LIDAR processing
- `detect_lanes.m` - Lane detection

---

## 🎮 Common Commands

```matlab
% Sensor Reading
[rgb, depth] = qcar.read_rgbd_front();    % Front RGB-D camera
lidar_data = qcar.read_lidar();           % LIDAR point cloud

% Vehicle Control
qcar.write_velocity(speed, steering);      % speed (m/s), steering (rad)

% Visualization
imshow(rgb);                               % Display camera image
scatter(lidar_data(:,1), lidar_data(:,2)); % Plot LIDAR points

% Status
qcar.get_status();                         % Check vehicle status
```

---

## 📝 File Naming Convention

| Type | Pattern | Example |
|------|---------|---------|
| Scripts | `snake_case.m` | `setup_qcar.m` |
| Functions | `snake_case.m` | `pid_controller.m` |
| Models | `snake_case.slx` | `vehicle_control.slx` |
| Test files | `test_*.m` | `test_steering.m` |
| Data files | `*.mat` | `calibration_data.mat` |

---

## ⚠️ Notes

- Always call `qcar.terminate()` before closing MATLAB
- Check CPS in QLabs settings if things are running slow
- Save work before running long simulations
- Use `try/catch` blocks to handle errors gracefully

---

*Beach Autonomous Systems - CSULB*
