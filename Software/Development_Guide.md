# 🛠️ Development Guide - ACC 2026

This guide documents the development workflow for the ACC 2026 Self-Driving Car Competition.

## Table of Contents

1. [Setup](#setup)
2. [MATLAB Development Workflow](#matlab-development-workflow)
3. [Testing in QLabs](#testing-in-qlabs)
4. [Best Practices](#best-practices)

---

## Setup

Using MATLAB because the ThinkPad L13 doesn't have an NVIDIA GPU, and MATLAB runs natively on Windows without Docker.

| Choice | Why |
|--------|-----|
| 🟦 **MATLAB** | Control systems, rapid prototyping, no GPU needed |

---

## MATLAB Development Workflow

### Environment Setup

#### Step 1: Install QLabs
1. Download from [Quanser Interactive Labs](https://www.quanser.com/digital/quanser-interactive-labs/)
2. Register at [Quanser Academic Portal](https://portal.quanser.com/Accounts/Register)
3. Install and log in with registered email

#### Step 2: Install MATLAB Add-on
1. Open MATLAB (R2023a or newer)
2. Go to **Home** → **Add-Ons** → **Get Add-Ons**
3. Search for "Quanser Interactive Labs for MATLAB"
4. Click **Add** to install

#### Step 3: Verify Installation
```matlab
% Set up QLabs connection (first time only)
QLabs.setup

% Launch QLabs
QLabs.launch

% Should see QLabs window open
```

---

### Daily Workflow

1. **Start Environment**
   ```matlab
   QLabs.launch          % Opens QLabs application
   ```

2. **Set Up Simulation**
   ```matlab
   % Navigate to Open Plane world in QLabs
   % Then in MATLAB:
   qcar = QCar2();
   qcar.spawn([0, 0, 0], 0);   % x, y, z position and heading
   ```

3. **Develop and Test**
   ```matlab
   % Read sensor data
   [front_rgb, front_depth] = qcar.read_rgbd_front();
   lidar_points = qcar.read_lidar();
   
   % Process and control
   steering = my_steering_algorithm(lidar_points);
   speed = 0.5;  % m/s
   
   % Send commands
   qcar.write_velocity(speed, steering);
   ```

4. **Clean Up**
   ```matlab
   qcar.terminate();
   ```

---

### Control System Example

```matlab
%% Main Control Loop
function main_control()
    % Initialize QCar
    qcar = QCar2();
    qcar.spawn([0, 0, 0], 0);
    
    % PID Controller gains
    Kp = 1.0;
    Ki = 0.1;
    Kd = 0.05;
    
    % Control loop
    dt = 0.05;  % 20 Hz
    integral_error = 0;
    prev_error = 0;
    
    try
        while true
            % Read sensors
            lidar_data = qcar.read_lidar();
            
            % Calculate error (e.g., lane center offset)
            error = calculate_lane_error(lidar_data);
            
            % PID control
            integral_error = integral_error + error * dt;
            derivative = (error - prev_error) / dt;
            steering = Kp * error + Ki * integral_error + Kd * derivative;
            
            % Clamp steering
            steering = max(-0.5, min(0.5, steering));
            
            % Apply control
            qcar.write_velocity(0.5, steering);
            
            prev_error = error;
            pause(dt);
        end
    catch ME
        disp(['Error: ', ME.message]);
    end
    
    % Cleanup
    qcar.terminate();
end
```

---

### Simulink Integration

Create Simulink models for complex control systems:

1. **Create New Model:** File → New → Simulink Model
2. **Add QCar Blocks:** Use MATLAB Function blocks to interface with QCar
3. **Design Controller:** Use Control System Toolbox for PID tuning
4. **Simulate:** Run in Simulink connected to QLabs

---

## Testing in QLabs

### Launching QLabs

```matlab
QLabs.launch              % Start QLabs
% Navigate to Open Plane in QLabs GUI
% Then spawn QCar:
qcar = QCar2();
qcar.spawn([0, 0, 0], 0);
```

### Testing Checklist

- [ ] `QLabs.launch` opens QLabs window
- [ ] QCar spawns at correct position
- [ ] Camera images display correctly
- [ ] LIDAR data is received
- [ ] Vehicle responds to velocity commands
- [ ] No significant lag (check CPS in QLabs settings)

### Debugging in MATLAB

```matlab
% Test sensor readings
[rgb, depth] = qcar.read_rgbd_front();
imshow(rgb);                    % Display camera image

lidar = qcar.read_lidar();
scatter(lidar(:,1), lidar(:,2)); % Plot LIDAR points

% Test control
qcar.write_velocity(0.3, 0);    % Drive straight
pause(2);
qcar.write_velocity(0, 0);      % Stop

% Check for errors
qcar.get_status()               % Get vehicle status
```

### Recording Data

```matlab
% Record sensor data
data = struct();
for i = 1:100
    data(i).time = datetime('now');
    data(i).lidar = qcar.read_lidar();
    [data(i).rgb, data(i).depth] = qcar.read_rgbd_front();
    pause(0.1);
end
save('test_run.mat', 'data');
```

---

## Best Practices

### Code Quality

1. **Use Comments**
   ```matlab
   % Calculate steering angle based on lane center offset
   steering = Kp * error + Ki * integral + Kd * derivative;
   ```

2. **Document Functions**
   ```matlab
   function detections = detect_objects(frame)
   % DETECT_OBJECTS - Detect objects in camera frame
   %
   %   Input:
   %       frame - RGB image as matrix
   %
   %   Output:
   %       detections - List of detected objects with bounding boxes
   ```

3. **Handle Errors**
   ```matlab
   try
       result = risky_operation();
   catch ME
       disp(['Error: ', ME.message]);
   end
   ```

### Git Practices

1. **Meaningful Commits**
   ```
   feat: Add traffic light detection
   fix: Correct steering angle calculation
   docs: Update README with setup instructions
   refactor: Simplify path planning logic
   ```

2. **Branch Strategy**
   ```
   main          # Stable, tested code
   ├── develop   # Integration branch
   ├── feature/* # New features
   └── fix/*     # Bug fixes
   ```

### Performance Optimization

1. **Profile Code**
   ```matlab
   profile on
   my_function();
   profile viewer
   ```

2. **Use Vectorized Operations**
   ```matlab
   % Instead of loops:
   distances = sqrt(sum(points.^2, 2));
   ```

---

## Resources

- [Quanser Interactive Labs for MATLAB](https://www.mathworks.com/matlabcentral/fileexchange/123860-quanser-interactive-labs-for-matlab)
- [QLabs MATLAB Documentation](https://qlabs.quanserdocs.com/)
- [Simulink Getting Started](https://www.mathworks.com/help/simulink/getting-started-with-simulink.html)
- [MATLAB Style Guide](https://www.mathworks.com/matlabcentral/fileexchange/46056-matlab-style-guidelines-2-0)

---

*Beach Autonomous Systems - CSULB*
