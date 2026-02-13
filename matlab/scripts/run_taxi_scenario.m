%% Run Taxi Scenario (Mission + LED) - ACC Virtual Detailed Scenario
% This script prepares the workspace for the taxi scenario.
%
% Assumptions:
% - You have already run the official environment script:
%     python/Setup_Real_Scenario_fullscale_x10.py
%   which spawns the QCar in the Taxi Hub Area.
% - You will control the QCar via QUARC/Simulink (competition compliant).

clear; clc;

% Resolve project root robustly (handles MATLAB Editor temp directory issue)
scriptDir = fileparts(mfilename('fullpath'));
projectRoot = '';

candidate = fullfile(scriptDir, '..', 'config', 'vehicle_params.m');
if exist(candidate, 'file')
    projectRoot = fullfile(scriptDir, '..');
end

if isempty(projectRoot)
    if exist(fullfile(pwd, 'matlab', 'config', 'vehicle_params.m'), 'file')
        projectRoot = fullfile(pwd, 'matlab');
    elseif exist(fullfile(pwd, 'config', 'vehicle_params.m'), 'file')
        projectRoot = pwd;
    elseif exist(fullfile(pwd, '..', 'config', 'vehicle_params.m'), 'file')
        projectRoot = fullfile(pwd, '..');
    end
end

if isempty(projectRoot)
    knownFcn = which('taxi_stack_sfun');
    if ~isempty(knownFcn)
        projectRoot = fullfile(fileparts(knownFcn), '..');
    end
end

if isempty(projectRoot)
    projectRoot = 'C:/Users/aguil/OneDrive/Desktop/ACC_2026/ACC-Competition-2026/matlab';
    if ~exist(fullfile(projectRoot, 'config', 'vehicle_params.m'), 'file')
        error(['Cannot locate project files. Please cd into the repo root or matlab/ directory, ' ...
               'or run this script from matlab/scripts/.']);
    end
end

addpath(fullfile(projectRoot, 'functions'));
addpath(fullfile(projectRoot, 'config'));

% Load parameters
run(fullfile(projectRoot, 'config', 'vehicle_params.m'));
run(fullfile(projectRoot, 'config', 'taxi_scenario_params.m'));

% Build the QUARC-integrated model (reads GPS, writes motor/steering/LED)
run(fullfile(projectRoot, 'scripts', 'build_taxi_quarc_model.m'));

% Open the model
open_system(fullfile(projectRoot, 'models', 'taxi_quarc.slx'));

fprintf('\nTaxi scenario parameters loaded into workspace variable `taxi`.\n');
fprintf('Pickup:  [%.3f, %.3f]\n', taxi.pickup_xy(1), taxi.pickup_xy(2));
fprintf('Dropoff: [%.3f, %.3f]\n', taxi.dropoff_xy(1), taxi.dropoff_xy(2));
fprintf('Hub:     [%.3f, %.3f]\n', taxi.hub_xy(1), taxi.hub_xy(2));
