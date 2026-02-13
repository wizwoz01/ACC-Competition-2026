%% Build Taxi Autonomy Stack Model (Algorithm Only)
% Creates a Simulink model that implements the taxi scenario mission FSM
% with pure pursuit waypoint following and lane-keeping correction.
%
% Output ports:
%   v_cmd_mps    - desired speed (m/s)
%   delta_cmd_rad- steering command (rad)
%   led_rgb      - uint8(3x1) LED color
%   state        - uint8 mission state
%
% This model is designed to be wired into QUARC I/O blocks in your own
% hardware-in-the-loop model (GPS/pose in, motor/steering/LED out).

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

% Load parameters into base workspace
run(fullfile(projectRoot, 'config', 'vehicle_params.m'));
run(fullfile(projectRoot, 'config', 'taxi_scenario_params.m'));

model = 'taxi_stack';

modelFile = fullfile(projectRoot, 'models', [model '.slx']);

if bdIsLoaded(model)
    close_system(model, 0);
end

% Ensure we don't accidentally open a stale saved copy.
if exist(modelFile, 'file')
    try
        delete(modelFile);
    catch
    end
end

new_system(model);
open_system(model);

% Basic layout constants
x0 = 50; y0 = 50; dx = 180; dy = 70;

% Inports
add_block('simulink/Sources/In1', [model '/pos_xy'], 'Position', [x0 y0 x0+30 y0+20]);
set_param([model '/pos_xy'], 'PortDimensions', '2');

add_block('simulink/Sources/In1', [model '/heading_rad'], 'Position', [x0 y0+dy x0+30 y0+dy+20]);

add_block('simulink/Sources/In1', [model '/speed_mps'], 'Position', [x0 y0+2*dy x0+30 y0+2*dy+20]);

add_block('simulink/Sources/Clock', [model '/t_sec'], 'Position', [x0 y0+3*dy x0+50 y0+3*dy+20]);

add_block('simulink/Sources/Constant', [model '/reset'], 'Position', [x0 y0+4*dy x0+50 y0+4*dy+20], 'Value', '0');

% Input 6: lane_offset (from camera lane detection, or 0 if no camera)
add_block('simulink/Sources/In1', [model '/lane_offset'], 'Position', [x0 y0+5*dy x0+30 y0+5*dy+20]);

% Mission + steering S-Function block (6 inputs, 4 outputs)
sfunPath = [model '/TaxiStack'];
pos = [x0+dx y0+dy x0+dx+200 y0+dy+160];

added = false;
libCandidates = { ...
    'simulink/User-Defined Functions/Level-2 MATLAB S-Function', ...
    'simulink/User-Defined Functions/Level-2 MATLAB S-Function (Obsolete)', ...
    'simulink/User-Defined Functions/Level-2 S-Function', ...
    'simulink/User-Defined Functions/MATLAB Level-2 S-Function' ...
    };

for i = 1:numel(libCandidates)
    try
        add_block(libCandidates{i}, sfunPath, 'Position', pos);
        added = true;
        break;
    catch
    end
end

if ~added
    error(['Could not add a Level-2 MATLAB S-Function block. ' ...
        'Open the Simulink Library Browser and confirm the exact path/name of the Level-2 MATLAB S-Function block in your version.']);
end

% Set the referenced MATLAB function name
set_level2_sfun_name(sfunPath, 'taxi_stack_sfun');

% Saturations
add_block('simulink/Discontinuities/Saturation', [model '/SatSpeed'], 'Position', [x0+3*dx y0+dy x0+3*dx+60 y0+dy+30]);
set_param([model '/SatSpeed'], 'LowerLimit', '0', 'UpperLimit', '1.0');

add_block('simulink/Discontinuities/Saturation', [model '/SatSteer'], 'Position', [x0+3*dx y0+2*dy x0+3*dx+60 y0+2*dy+30]);
set_param([model '/SatSteer'], 'LowerLimit', '-0.5', 'UpperLimit', '0.5');

% Outports
add_block('simulink/Sinks/Out1', [model '/v_cmd_mps'], 'Position', [x0+4*dx y0+dy x0+4*dx+30 y0+dy+20]);
add_block('simulink/Sinks/Out1', [model '/delta_cmd_rad'], 'Position', [x0+4*dx y0+2*dy x0+4*dx+30 y0+2*dy+20]);
add_block('simulink/Sinks/Out1', [model '/led_rgb'], 'Position', [x0+4*dx y0+3*dy x0+4*dx+30 y0+3*dy+20]);
add_block('simulink/Sinks/Out1', [model '/state'], 'Position', [x0+4*dx y0+4*dy x0+4*dx+30 y0+4*dy+20]);

% Wiring (6 inputs)
add_line(model, 'pos_xy/1', 'TaxiStack/1');
add_line(model, 'heading_rad/1', 'TaxiStack/2');
add_line(model, 'speed_mps/1', 'TaxiStack/3');
add_line(model, 't_sec/1', 'TaxiStack/4');
add_line(model, 'reset/1', 'TaxiStack/5');
add_line(model, 'lane_offset/1', 'TaxiStack/6');

add_line(model, 'TaxiStack/1', 'SatSpeed/1');
add_line(model, 'TaxiStack/2', 'SatSteer/1');

add_line(model, 'SatSpeed/1', 'v_cmd_mps/1');
add_line(model, 'SatSteer/1', 'delta_cmd_rad/1');
add_line(model, 'TaxiStack/3', 'led_rgb/1');
add_line(model, 'TaxiStack/4', 'state/1');

% Model settings
set_param(model, 'StopTime', 'inf');
set_param(model, 'SolverType', 'Fixed-step');
set_param(model, 'FixedStep', num2str(params.control.dt));

save_system(model, modelFile);

fprintf('Created %s\n', fullfile('matlab', 'models', [model '.slx']));
fprintf('Inputs: pos_xy, heading_rad, speed_mps, t_sec (Clock), reset, lane_offset\n');
fprintf('Outputs: v_cmd_mps, delta_cmd_rad, led_rgb, state\n');
fprintf('Wire pos_xy/heading/speed from your localization (e.g., GPS/IMU).\n');
fprintf('Wire lane_offset from camera lane detection (or use Constant(0) for waypoint-only).\n');
fprintf('Wire v_cmd_mps/delta_cmd_rad/led_rgb to your QUARC outputs.\n');

function set_level2_sfun_name(blockPath, matlabFcnName)
%SET_LEVEL2_SFUN_NAME Set the MATLAB function/file name on a Level-2 MATLAB S-Function block.

    dp = struct();
    try
        dp = get_param(blockPath, 'DialogParameters');
    catch
    end

    candidates = {};
    names = {};
    try
        names = fieldnames(dp);
    catch
        names = {};
    end

    if ~isempty(names)
        if any(strcmp(names, 'FunctionName'))
            candidates = {'FunctionName'};
        elseif any(strcmp(names, 'MATLABFcn'))
            candidates = {'MATLABFcn'};
        elseif any(strcmp(names, 'MATLABFile'))
            candidates = {'MATLABFile'};
        else
            candidates = {'FunctionName', 'MATLABFcn', 'MATLABFile'};
        end
    else
        candidates = {'FunctionName', 'MATLABFcn', 'MATLABFile'};
    end

    setOk = false;
    lastErr = [];

    for i = 1:numel(candidates)
        p = candidates{i};
        try
            set_param(blockPath, p, matlabFcnName);
            setOk = true;
            break;
        catch err
            lastErr = err;
        end
    end

    if ~setOk
        try
            maskNames = get_param(blockPath, 'MaskNames');
            maskValues = get_param(blockPath, 'MaskValues');
            idx = find(strcmp(maskNames, 'FunctionName'), 1);
            if isempty(idx)
                idx = find(strcmpi(maskNames, 'FunctionName'), 1);
            end
            if isempty(idx)
                for k = 1:numel(maskNames)
                    if ~isempty(strfind(lower(maskNames{k}), 'function'))
                        idx = k;
                        break;
                    end
                end
            end
            if ~isempty(idx)
                maskValues{idx} = matlabFcnName;
                set_param(blockPath, 'MaskValues', maskValues);
                setOk = true;
            end
        catch err
            lastErr = err;
        end
    end

    if ~setOk
        avail = '';
        try
            availNames = fieldnames(dp);
            for i = 1:numel(availNames)
                avail = sprintf('%s\n  - %s', avail, availNames{i});
            end
        catch
        end

        mAvail = '';
        try
            mn = get_param(blockPath, 'MaskNames');
            for i = 1:numel(mn)
                mAvail = sprintf('%s\n  - %s', mAvail, mn{i});
            end
        catch
        end

        if ~isempty(lastErr)
            error(['Failed to set Level-2 MATLAB S-Function name on block: ' blockPath '\n' ...
                'Tried parameters: ' strjoin(candidates, ', ') '\n' ...
                'Available dialog parameters (if any):' avail '\n' ...
                'Available mask names (if any):' mAvail '\n' ...
                'Last error: ' lastErr.message]);
        end

        error(['Failed to set Level-2 MATLAB S-Function name on block: ' blockPath '\n' ...
            'Tried parameters: ' strjoin(candidates, ', ') '\n' ...
            'Available dialog parameters (if any):' avail '\n' ...
            'Available mask names (if any):' mAvail]);
    end
end
