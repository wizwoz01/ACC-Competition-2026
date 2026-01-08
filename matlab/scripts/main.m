%% Main Entry Point - ACC 2026 Self-Driving Car Competition
% Beach Autonomous Systems - CSULB
%
% This script initializes the QCar and runs the main control loop.

%% Clear workspace
clear; clc; close all;

%% Add paths
addpath('../functions');

%% Configuration
config.speed = 0.5;              % Default speed (m/s)
config.dt = 0.05;                % Control loop period (20 Hz)
config.max_steering = 0.5;       % Max steering angle (rad)

%% Initialize QLabs and QCar
fprintf('Launching QLabs...\n');
QLabs.launch;
pause(3);  % Wait for QLabs to open

fprintf('Initializing QCar...\n');
qcar = QCar2();
qcar.spawn([0, 0, 0], 0);  % Spawn at origin with 0 heading
fprintf('QCar spawned successfully!\n');

%% Main Control Loop
fprintf('Starting control loop...\n');
try
    while true
        % Read sensors
        lidar_data = qcar.read_lidar();
        [rgb, depth] = qcar.read_rgbd_front();
        
        % Process perception
        % TODO: Add perception processing
        
        % Calculate control
        steering = 0;  % TODO: Add steering calculation
        speed = config.speed;
        
        % Apply control
        qcar.write_velocity(speed, steering);
        
        % Control loop timing
        pause(config.dt);
    end
catch ME
    fprintf('Error: %s\n', ME.message);
end

%% Cleanup
fprintf('Shutting down...\n');
qcar.terminate();
fprintf('Done!\n');

