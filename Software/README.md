# 💻 Software Development Guide - ACC 2026

This directory contains development guides and utilities for the ACC 2026 Self-Driving Car Competition.

## Contents

- [Development Guide](Development_Guide.md) - Main development workflow
- [QLabs Guide](QLabs_Guide.md) - Quanser Interactive Labs usage

---

## 🟦 MATLAB/Simulink Stack

Using MATLAB because it doesn't require an NVIDIA GPU and runs natively on Windows.

| Component | Technology |
|-----------|------------|
| OS | Windows 10/11 (native) |
| Development | MATLAB R2023a+ / Simulink |
| Simulation | Quanser Interactive Labs (QLabs) |
| Add-on | Quanser Interactive Labs for MATLAB |
| Language | MATLAB / Simulink blocks |

---

## Quick Reference

### MATLAB Commands

```matlab
% First-time setup
QLabs.setup

% Launch QLabs from MATLAB
QLabs.launch

% Spawn QCar 2 in simulation
qcar = QCar2();                    % Create QCar object
qcar.spawn([0, 0, 0], 0);          % Spawn at position with heading

% Read sensors
[rgb, depth] = qcar.read_cameras();
lidar_data = qcar.read_lidar();

% Control vehicle
qcar.write_velocity(speed, steering_angle);

% Stop and cleanup
qcar.terminate();
```

---

## Development Workflow

### 🟦 MATLAB Workflow

1. **Start Environment**
   - Launch MATLAB
   - Run `QLabs.launch` to start simulation

2. **Develop**
   - Create Simulink models for control systems
   - Write MATLAB scripts for algorithms
   - Use MATLAB's debugging tools

3. **Build & Test**
   - Run simulations in QLabs
   - Use Simulink's simulation mode
   - Tune parameters in real-time

4. **Iterate**
   - Analyze data with MATLAB plots
   - Refine control gains
   - Optimize algorithms

---

## Project Structure

### 🟦 MATLAB Structure

```
matlab/
├── models/               # Simulink models
│   ├── main_controller.slx
│   ├── path_planner.slx
│   └── perception_pipeline.slx
├── scripts/              # MATLAB scripts
│   ├── main.m
│   ├── setup_qcar.m
│   ├── lane_detection.m
│   └── path_planning.m
├── functions/            # Reusable functions
│   ├── pid_controller.m
│   ├── pure_pursuit.m
│   └── image_processing.m
└── config/               # Parameters
    └── vehicle_params.m
```

---

## Tools

### 🟦 MATLAB Tools
- **Simulink** - Model-based design
- **Stateflow** - State machines for decision logic
- **Computer Vision Toolbox** - Image processing
- **Control System Toolbox** - Controller design
- **MATLAB Plots** - Data visualization

---

## Resources

### MATLAB Resources
- [Quanser Interactive Labs for MATLAB](https://www.mathworks.com/matlabcentral/fileexchange/123860-quanser-interactive-labs-for-matlab)
- [QLabs MATLAB Documentation](https://qlabs.quanserdocs.com/)
- [Simulink Getting Started](https://www.mathworks.com/help/simulink/getting-started-with-simulink.html)

---

*Beach Autonomous Systems - CSULB*
