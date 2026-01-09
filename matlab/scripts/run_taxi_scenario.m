%% Run Taxi Scenario (Mission + LED) - ACC Virtual Detailed Scenario
% This script prepares the workspace for the taxi scenario.
%
% Assumptions:
% - You have already run the official environment script:
%     python/Setup_Real_Scenario.py
%   which spawns the QCar in the Taxi Hub Area.
% - You will control the QCar via QUARC/Simulink (competition compliant).

clear; clc;

scriptDir = fileparts(mfilename('fullpath'));
addpath(fullfile(scriptDir, '..', 'functions'));
addpath(fullfile(scriptDir, '..', 'config'));

% Load parameters
run(fullfile(scriptDir, '..', 'config', 'vehicle_params.m'));
run(fullfile(scriptDir, '..', 'config', 'taxi_scenario_params.m'));

% Build the QUARC-integrated model (reads GPS, writes motor/steering/LED)
run(fullfile(scriptDir, 'build_taxi_quarc_model.m'));

% Open the model
open_system(fullfile(scriptDir, '..', 'models', 'taxi_quarc.slx'));

fprintf('\nTaxi scenario parameters loaded into workspace variable `taxi`.\n');
fprintf('Pickup:  [%.3f, %.3f]\n', taxi.pickup_xy(1), taxi.pickup_xy(2));
fprintf('Dropoff: [%.3f, %.3f]\n', taxi.dropoff_xy(1), taxi.dropoff_xy(2));
fprintf('Hub:     [%.3f, %.3f]\n', taxi.hub_xy(1), taxi.hub_xy(2));
