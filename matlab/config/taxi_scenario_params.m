%% Taxi Scenario Parameters (ACC Virtual Detailed Scenario)
% Defines mission coordinates and thresholds for the taxi pick-up / drop-off scenario.
%
% Usage:
%   run('matlab/config/taxi_scenario_params.m');
%   % creates variable `taxi` in the workspace

% Coordinates are in meters in the competition base frame.
% Taxi hub pose comes from python/Setup_Real_Scenario.py initialPosition.

taxi = struct();

% Competition full-scale coordinate system
% If you are running python/Setup_Real_Scenario_fullscale_x10.py, positions
% published by the scenario (and GPS) are scaled by 10.
taxi.coord_scale = 10.0;

% Key locations (x,y)
taxi.hub_xy     = [-1.205; -0.830];
taxi.pickup_xy  = [ 0.125;  4.395];
taxi.dropoff_xy = [-0.905;  0.800];

% Mission behavior
% How close is "arrived" to a coordinate?
taxi.arrival_radius_m = 0.30;

% How long to hold a full stop at pickup/dropoff/hub (seconds)
taxi.stop_hold_sec = 2.0;

% Optional: show red briefly before moving (seconds)
taxi.init_hold_sec = 5.0;

% Speed profile
% Cruise speed while navigating (m/s)
taxi.cruise_speed_mps = 0.50;

% Speed threshold to consider the vehicle "stopped" (m/s)
taxi.stop_speed_thresh_mps = 0.05;

% LED colors (uint8 RGB 0..255)
taxi.led_red    = uint8([255; 0;   0]);
taxi.led_green  = uint8([0;   255; 0]);
taxi.led_blue   = uint8([0;   0;   255]);
% Orange / amber
% (If your LED driver expects different scaling, convert at the I/O boundary.)
taxi.led_orange = uint8([255; 165; 0]);
