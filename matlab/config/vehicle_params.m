%% Vehicle Parameters - QCar 2
% Beach Autonomous Systems - CSULB
%
% This file contains QCar 2 vehicle parameters for use in control algorithms.

%% Physical Parameters
vehicle.wheelbase = 0.256;          % Wheelbase (m)
vehicle.track_width = 0.17;         % Track width (m)
vehicle.max_steering = 0.5;         % Max steering angle (rad)
vehicle.max_speed = 1.0;            % Max speed (m/s)

%% Control Parameters
control.dt = 0.05;                  % Control loop period (s) - 20 Hz
control.speed_default = 0.5;        % Default cruise speed (m/s)
control.speed_slow = 0.3;           % Slow speed for turns/obstacles (m/s)

%% PID Gains - Steering
pid.steering.Kp = 1.0;
pid.steering.Ki = 0.1;
pid.steering.Kd = 0.05;

%% PID Gains - Speed
pid.speed.Kp = 0.5;
pid.speed.Ki = 0.1;
pid.speed.Kd = 0.01;

%% Pure Pursuit Parameters
pursuit.lookahead_distance = 0.5;   % Lookahead distance (m)
pursuit.min_lookahead = 0.3;        % Minimum lookahead (m)
pursuit.max_lookahead = 1.0;        % Maximum lookahead (m)

%% Sensor Configuration
sensors.lidar.min_range = 0.1;      % Minimum valid range (m)
sensors.lidar.max_range = 10.0;     % Maximum valid range (m)
sensors.camera.width = 820;         % Camera width (pixels)
sensors.camera.height = 410;        % Camera height (pixels)

%% Lane Detection Parameters
lanes.roi_top = 0.5;                % ROI top percentage
lanes.white_thresh = 200;           % White lane threshold
lanes.offset_gain = 1.5;            % Lane offset to steering gain

%% Safety Parameters
safety.min_obstacle_dist = 0.5;     % Minimum obstacle distance (m)
safety.emergency_stop_dist = 0.3;   % Emergency stop distance (m)
safety.collision_radius = 0.15;     % Vehicle collision radius (m)

%% Export as struct for easy passing
params = struct();
params.vehicle = vehicle;
params.control = control;
params.pid = pid;
params.pursuit = pursuit;
params.sensors = sensors;
params.lanes = lanes;
params.safety = safety;

fprintf('Vehicle parameters loaded.\n');

