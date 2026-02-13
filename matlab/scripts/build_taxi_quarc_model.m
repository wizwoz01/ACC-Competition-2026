%% Build Complete Taxi Scenario Model (QUARC + Algorithm + Lane Detection)
% Creates a Simulink model that:
%   1) Reads GPS position from QLabs QCar via QUARC
%   2) Reads front camera image for lane detection
%   3) Runs lane detection to produce lane center offset
%   4) Runs the taxi mission FSM + pure pursuit steering + lane keeping
%   5) Writes motor throttle, steering, and LED commands via QUARC
%
% Prerequisites:
%   - QLabs running with Cityscape workspace
%   - python/Setup_Real_Scenario_fullscale_x10.py running (spawns QCar + traffic lights)
%   - QUARC toolbox installed

clear; clc;

% -------------------------------------------------------------------------
% QUARC HIL availability probe
% -------------------------------------------------------------------------
useHIL = true;
try
    load_system('quarc_library');
    probeModel = '__taxi_quarc_hil_probe__';
    if bdIsLoaded(probeModel)
        close_system(probeModel, 0);
    end
    new_system(probeModel);

    hilLibCandidates = {
        'quarc_library/Data Acquisition/Generic/Configuration/HIL Initialize',
        'quarc_library/HIL Initialize'
    };
    added = false;
    for i = 1:numel(hilLibCandidates)
        try
            add_block(hilLibCandidates{i}, [probeModel '/HIL_Initialize'], 'Position', [30 30 220 90]);
            added = true;
            break;
        catch
        end
    end

    if ~added
        useHIL = false;
    else
        boardTypeNames = {'board_type', 'BoardType', 'Board', 'board'};
        boardIdNames = {'board_identifier', 'BoardIdentifier', 'Identifier', 'board_id', 'BoardId', 'URI', 'uri'};
        for i = 1:numel(boardTypeNames)
            try
                set_param([probeModel '/HIL_Initialize'], boardTypeNames{i}, 'qcar2');
                break;
            catch
            end
        end
        for i = 1:numel(boardIdNames)
            try
                set_param([probeModel '/HIL_Initialize'], boardIdNames{i}, '0@tcpip://localhost:18960');
                break;
            catch
            end
        end
        set_param(probeModel, 'SimulationCommand', 'update');
    end

    close_system(probeModel, 0);
    try, close_system('quarc_library', 0); catch, end
catch
    useHIL = false;
    try, close_system('__taxi_quarc_hil_probe__', 0); catch, end
    try, close_system('quarc_library', 0); catch, end
end

if ~useHIL
    warning(['QCar2 QUARC HIL support not detected. ' ...
             'Building taxi_quarc in algorithm-only mode (no HIL blocks). ' ...
             'To run the full QLabs scenario via QUARC, register/install QUARC with QCar2 support.']);
end

% Resolve project root robustly (handles MATLAB Editor temp directory issue)
scriptDir = fileparts(mfilename('fullpath'));
projectRoot = '';

% Strategy 1: mfilename path (works when run from command window)
candidate = fullfile(scriptDir, '..', 'config', 'vehicle_params.m');
if exist(candidate, 'file')
    projectRoot = fullfile(scriptDir, '..');
end

% Strategy 2: look relative to current working directory
if isempty(projectRoot)
    % Try: pwd is the repo root
    if exist(fullfile(pwd, 'matlab', 'config', 'vehicle_params.m'), 'file')
        projectRoot = fullfile(pwd, 'matlab');
    % Try: pwd is matlab/
    elseif exist(fullfile(pwd, 'config', 'vehicle_params.m'), 'file')
        projectRoot = pwd;
    % Try: pwd is matlab/scripts/
    elseif exist(fullfile(pwd, '..', 'config', 'vehicle_params.m'), 'file')
        projectRoot = fullfile(pwd, '..');
    end
end

% Strategy 3: use 'which' to find a known function on the path
if isempty(projectRoot)
    knownFcn = which('taxi_stack_sfun');
    if ~isempty(knownFcn)
        projectRoot = fullfile(fileparts(knownFcn), '..');
    end
end

% Strategy 4: hardcoded fallback
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

model = 'taxi_quarc';
modelFile = fullfile(projectRoot, 'models', [model '.slx']);

if bdIsLoaded(model)
    close_system(model, 0);
end

if exist(modelFile, 'file')
    try
        delete(modelFile);
    catch
    end
end

new_system(model);
open_system(model);

% Layout constants
x0 = 50; y0 = 50; dx = 200; dy = 80;

%% ==================== HIL Initialize ====================
hilInitPath = [model '/HIL_Initialize'];
hilInitOk = false;
if useHIL
    try
        add_block('quarc_library/Data Acquisition/Generic/Configuration/HIL Initialize', hilInitPath, ...
            'Position', [x0 y0 x0+150 y0+60]);
        hilInitOk = true;
    catch
        try
            add_block('quarc_library/HIL Initialize', hilInitPath, ...
                'Position', [x0 y0 x0+150 y0+60]);
            hilInitOk = true;
        catch
        end
    end

    if hilInitOk
        paramSet = false;
        boardTypeNames = {'board_type', 'BoardType', 'Board', 'board'};
        boardIdNames = {'board_identifier', 'BoardIdentifier', 'Identifier', 'board_id', 'BoardId', 'URI', 'uri'};

        for i = 1:numel(boardTypeNames)
            try
                set_param(hilInitPath, boardTypeNames{i}, 'qcar2');
                paramSet = true;
                break;
            catch
            end
        end

        for i = 1:numel(boardIdNames)
            try
                set_param(hilInitPath, boardIdNames{i}, '0@tcpip://localhost:18960');
                break;
            catch
            end
        end

        if ~paramSet
            warning('Could not set HIL Initialize parameters. You may need to configure the block manually.');
        end
    else
        warning('Could not add HIL Initialize block. Adding placeholder.');
        add_block('simulink/Sources/Constant', hilInitPath, 'Position', [x0 y0 x0+80 y0+40], 'Value', '0');
    end
else
    add_block('simulink/Sources/Constant', hilInitPath, 'Position', [x0 y0 x0+80 y0+40], 'Value', '0');
end

% Terminate HIL Initialize output
try
    termPath = [model '/HIL_Init_Term'];
    add_block('simulink/Sinks/Terminator', termPath, 'Position', [x0+170 y0+15 x0+190 y0+35]);
    add_line(model, 'HIL_Initialize/1', 'HIL_Init_Term/1');
catch
end

%% ==================== GPS Read ====================
gpsPath = [model '/GPS_Read'];
gpsOk = false;
try
    add_block('quarc_library/Communications/UDP/Stream Client/Stream Read', gpsPath, ...
        'Position', [x0 y0+2*dy x0+120 y0+2*dy+60]);
    gpsOk = true;
catch
    try
        add_block('quarc_library/Stream Read', gpsPath, ...
            'Position', [x0 y0+2*dy x0+120 y0+2*dy+60]);
        gpsOk = true;
    catch
    end
end

if gpsOk
    uriNames = {'uri', 'URI', 'Address', 'address', 'Url', 'url'};
    for i = 1:numel(uriNames)
        try
            set_param(gpsPath, uriNames{i}, 'tcpip://localhost:18967');
            break;
        catch
        end
    end
else
    warning('Could not add GPS Stream Read block. Using simulated GPS.');
    add_block('simulink/Sources/Constant', gpsPath, 'Position', [x0 y0+2*dy x0+100 y0+2*dy+40]);
    coord_scale = 1.0;
    try
        if exist('taxi', 'var') && isstruct(taxi) && isfield(taxi, 'coord_scale')
            coord_scale = double(taxi.coord_scale);
        end
    catch
    end
    if ~isfinite(coord_scale) || coord_scale <= 0
        coord_scale = 1.0;
    end
    hub0 = [-1.205, -0.830] * coord_scale;
    set_param(gpsPath, 'Value', sprintf('[%.6f %.6f 0 0 0 -0.78]', hub0(1), hub0(2)));
    set_param(gpsPath, 'OutDataTypeStr', 'double');
    set_param(gpsPath, 'SampleTime', '0.01');
end

%% ==================== Extract Position and Heading ====================
posSelPath = [model '/Pos_XY_Selector'];
add_block('simulink/Signal Routing/Selector', posSelPath, 'Position', [x0+dx y0+2*dy x0+dx+60 y0+2*dy+40]);
set_param(posSelPath, 'NumberOfDimensions', '1');
set_param(posSelPath, 'IndexMode', 'One-based');
set_param(posSelPath, 'InputPortWidth', '6');
set_param(posSelPath, 'IndexOptions', 'Index vector (dialog)');
set_param(posSelPath, 'Indices', '[1 2]');
set_param(posSelPath, 'OutputSizes', '2');

headSelPath = [model '/Heading_Selector'];
add_block('simulink/Signal Routing/Selector', headSelPath, 'Position', [x0+dx y0+2*dy+50 x0+dx+60 y0+2*dy+90]);
set_param(headSelPath, 'NumberOfDimensions', '1');
set_param(headSelPath, 'IndexMode', 'One-based');
set_param(headSelPath, 'InputPortWidth', '6');
set_param(headSelPath, 'IndexOptions', 'Index vector (dialog)');
set_param(headSelPath, 'Indices', '6');
set_param(headSelPath, 'OutputSizes', '1');

%% ==================== Clock and Reset ====================
add_block('simulink/Sources/Clock', [model '/Clock'], 'Position', [x0 y0+4*dy x0+50 y0+4*dy+30]);
add_block('simulink/Sources/Constant', [model '/Reset'], 'Position', [x0 y0+5*dy x0+50 y0+5*dy+30], 'Value', '0');

%% ==================== Speed Estimate ====================
add_block('simulink/Sources/Constant', [model '/Speed_Est'], 'Position', [x0 y0+3*dy x0+80 y0+3*dy+30], 'Value', '0.1');

%% ==================== Camera Read (Front Camera) ====================
% The front camera is on port 18942. Try QUARC Video3D / Stream blocks.
camPath = [model '/Camera_Read'];
camOk = false;

% Try QUARC Video3D Capture block (multiple library path candidates)
if useHIL
    camLibCandidates = {
        'quarc_library/Video3D/Video3D Capture',
        'quarc_library/Devices/Cameras/Video3D Capture',
        'quarc_library/Communications/UDP/Stream Client/Stream Read'
    };
    for i = 1:numel(camLibCandidates)
        try
            add_block(camLibCandidates{i}, camPath, ...
                'Position', [x0 y0+7*dy x0+140 y0+7*dy+60]);
            camOk = true;
            % Try to configure for front camera
            uriNames = {'uri', 'URI', 'Address', 'address', 'Url', 'url'};
            for j = 1:numel(uriNames)
                try
                    set_param(camPath, uriNames{j}, 'tcpip://localhost:18942');
                    break;
                catch
                end
            end
            break;
        catch
        end
    end
end

if ~camOk
    warning(['Could not add Camera Read block. Using zero lane offset (waypoint-only mode). ' ...
             'To enable camera-based lane keeping, manually add a Video3D Capture block ' ...
             'reading port 18942, wire it through lane_detect_sfun, and connect to TaxiStack input 6.']);
    % Use Constant(0) as fallback - waypoint following will still work
    % SampleTime left at default (inf = constant) to avoid rate mismatch
    add_block('simulink/Sources/Constant', camPath, ...
        'Position', [x0 y0+7*dy x0+80 y0+7*dy+30], 'Value', '0');
    set_param(camPath, 'OutDataTypeStr', 'double');
end

%% ==================== Lane Detection S-Function ====================
% Only add if camera is available; otherwise, wire Constant(0) directly
laneDetPath = [model '/Lane_Detect'];
useLaneSfun = false;

if camOk
    % Add the lane detection S-function
    laneLibCandidates = { ...
        'simulink/User-Defined Functions/Level-2 MATLAB S-Function', ...
        'simulink/User-Defined Functions/Level-2 MATLAB S-Function (Obsolete)', ...
        'simulink/User-Defined Functions/Level-2 S-Function', ...
        'simulink/User-Defined Functions/MATLAB Level-2 S-Function' ...
    };
    for i = 1:numel(laneLibCandidates)
        try
            add_block(laneLibCandidates{i}, laneDetPath, ...
                'Position', [x0+dx y0+7*dy x0+dx+160 y0+7*dy+50]);
            useLaneSfun = true;
            break;
        catch
        end
    end

    if useLaneSfun
        % Set the function name to our lane_detect_sfun
        sfunNameCandidates = {'FunctionName', 'MATLABFcn', 'MATLABFile'};
        for i = 1:numel(sfunNameCandidates)
            try
                set_param(laneDetPath, sfunNameCandidates{i}, 'lane_detect_sfun');
                break;
            catch
            end
        end
    end
end

if ~useLaneSfun && camOk
    % Camera available but couldn't add S-function - use Constant(0)
    add_block('simulink/Sources/Constant', laneDetPath, ...
        'Position', [x0+dx y0+7*dy x0+dx+80 y0+7*dy+30], 'Value', '0');
    set_param(laneDetPath, 'OutDataTypeStr', 'double');
end

%% ==================== Lane Offset Source ====================
% Determine what provides the lane_offset signal to TaxiStack
% If camera + lane detection: Camera -> LaneDetect -> TaxiStack/6
% If no camera: Constant(0) -> TaxiStack/6

%% ==================== Taxi Stack S-Function (6 inputs) ====================
sfunPath = [model '/TaxiStack'];
pos = [x0+2*dx y0+2*dy x0+2*dx+200 y0+2*dy+160];

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
    error('Could not add a Level-2 MATLAB S-Function block.');
end

% Set the function name
try
    set_param(sfunPath, 'FunctionName', 'taxi_stack_sfun');
catch
end

%% ==================== Saturations ====================
add_block('simulink/Discontinuities/Saturation', [model '/SatSpeed'], 'Position', [x0+3*dx y0+2*dy x0+3*dx+60 y0+2*dy+30]);
set_param([model '/SatSpeed'], 'LowerLimit', '-0.3', 'UpperLimit', '0.3');

add_block('simulink/Discontinuities/Saturation', [model '/SatSteer'], 'Position', [x0+3*dx y0+2*dy+50 x0+3*dx+60 y0+2*dy+80]);
set_param([model '/SatSteer'], 'LowerLimit', '-0.5', 'UpperLimit', '0.5');

%% ==================== HIL Write (Motor + Steering) ====================
hilWritePath = [model '/HIL_Write'];
hilWriteOk = false;
if useHIL
    try
        add_block('quarc_library/Data Acquisition/Generic/Immediate I-O/HIL Write', hilWritePath, ...
            'Position', [x0+4*dx y0+2*dy x0+4*dx+120 y0+2*dy+80]);
        hilWriteOk = true;
    catch
        try
            add_block('quarc_library/HIL Write', hilWritePath, ...
                'Position', [x0+4*dx y0+2*dy x0+4*dx+120 y0+2*dy+80]);
            hilWriteOk = true;
        catch
        end
    end

    if hilWriteOk
        try set_param(hilWritePath, 'pwm_channels', '[1000]'); catch, end
        try set_param(hilWritePath, 'PWMChannels', '[1000]'); catch, end
        try set_param(hilWritePath, 'other_channels', '[0]'); catch, end
        try set_param(hilWritePath, 'OtherChannels', '[0]'); catch, end
    else
        warning('Could not add HIL Write block. Using Display instead.');
        add_block('simulink/Sinks/Display', hilWritePath, 'Position', [x0+4*dx y0+2*dy x0+4*dx+80 y0+2*dy+60]);
    end
else
    add_block('simulink/Sinks/Display', hilWritePath, 'Position', [x0+4*dx y0+2*dy x0+4*dx+80 y0+2*dy+60]);
end

%% ==================== LED Strip Write ====================
ledPath = [model '/LED_Write'];
ledOk = false;
try
    add_block('quarc_library/Communications/UDP/Stream Client/Stream Write', ledPath, ...
        'Position', [x0+4*dx y0+4*dy x0+4*dx+120 y0+4*dy+60]);
    ledOk = true;
catch
    try
        add_block('quarc_library/Stream Write', ledPath, ...
            'Position', [x0+4*dx y0+4*dy x0+4*dx+120 y0+4*dy+60]);
        ledOk = true;
    catch
    end
end

if ledOk
    uriNames = {'uri', 'URI', 'Address', 'address', 'Url', 'url'};
    for i = 1:numel(uriNames)
        try
            set_param(ledPath, uriNames{i}, 'tcpip://localhost:18969');
            break;
        catch
        end
    end
else
    warning('Could not add LED Stream Write block. Using Display instead.');
    add_block('simulink/Sinks/Display', ledPath, 'Position', [x0+4*dx y0+4*dy x0+4*dx+80 y0+4*dy+40]);
end

%% ==================== State Display ====================
add_block('simulink/Sinks/Display', [model '/State_Display'], 'Position', [x0+4*dx y0+5*dy x0+4*dx+60 y0+5*dy+30]);

%% ==================== Scopes for debugging ====================
add_block('simulink/Sinks/Scope', [model '/Position_Scope'], 'Position', [x0+3*dx y0+dy x0+3*dx+40 y0+dy+40]);
add_block('simulink/Sinks/Scope', [model '/Commands_Scope'], 'Position', [x0+4*dx y0+dy x0+4*dx+40 y0+dy+40]);

%% ==================== Lane Offset Scope ====================
add_block('simulink/Sinks/Scope', [model '/LaneOffset_Scope'], 'Position', [x0+3*dx y0+7*dy x0+3*dx+40 y0+7*dy+40]);

%% ==================== Mux for commands ====================
add_block('simulink/Signal Routing/Mux', [model '/Cmd_Mux'], 'Position', [x0+3.5*dx y0+2*dy+20 x0+3.5*dx+10 y0+2*dy+70]);
set_param([model '/Cmd_Mux'], 'Inputs', '2');

%% ==================== Wiring ====================
% GPS -> Selectors
add_line(model, 'GPS_Read/1', 'Pos_XY_Selector/1');
add_line(model, 'GPS_Read/1', 'Heading_Selector/1');

% Inputs -> TaxiStack (6 inputs)
add_line(model, 'Pos_XY_Selector/1', 'TaxiStack/1');    % pos_xy
add_line(model, 'Heading_Selector/1', 'TaxiStack/2');    % heading_rad
add_line(model, 'Speed_Est/1', 'TaxiStack/3');           % speed_mps
add_line(model, 'Clock/1', 'TaxiStack/4');               % t
add_line(model, 'Reset/1', 'TaxiStack/5');               % reset

% Lane offset -> TaxiStack input 6
if camOk && useLaneSfun
    % Camera -> LaneDetect -> TaxiStack/6
    add_line(model, 'Camera_Read/1', 'Lane_Detect/1');
    add_line(model, 'Lane_Detect/1', 'TaxiStack/6');
    % Also connect to scope for debugging
    add_line(model, 'Lane_Detect/1', 'LaneOffset_Scope/1');
elseif camOk
    % Camera available but no S-function: use constant(0)
    add_line(model, 'Lane_Detect/1', 'TaxiStack/6');
    add_line(model, 'Lane_Detect/1', 'LaneOffset_Scope/1');
else
    % No camera: Constant(0) -> TaxiStack/6
    add_line(model, 'Camera_Read/1', 'TaxiStack/6');
    add_line(model, 'Camera_Read/1', 'LaneOffset_Scope/1');
end

% TaxiStack -> Saturations
add_line(model, 'TaxiStack/1', 'SatSpeed/1');
add_line(model, 'TaxiStack/2', 'SatSteer/1');

% Saturations -> Mux -> HIL Write
add_line(model, 'SatSpeed/1', 'Cmd_Mux/1');
add_line(model, 'SatSteer/1', 'Cmd_Mux/2');
add_line(model, 'Cmd_Mux/1', 'HIL_Write/1');

% LED output
add_line(model, 'TaxiStack/3', 'LED_Write/1');

% State display
add_line(model, 'TaxiStack/4', 'State_Display/1');

% Scopes
add_line(model, 'Pos_XY_Selector/1', 'Position_Scope/1');
add_line(model, 'Cmd_Mux/1', 'Commands_Scope/1');

%% ==================== Model Settings ====================
set_param(model, 'StopTime', 'inf');
set_param(model, 'SolverType', 'Fixed-step');
set_param(model, 'FixedStep', '0.01');

%% ==================== Save ====================
save_system(model, modelFile);

fprintf('\n========================================\n');
fprintf('Created: matlab/models/taxi_quarc.slx\n');
fprintf('========================================\n\n');
fprintf('FEATURES:\n');
fprintf('- Pure pursuit waypoint following (road-following paths)\n');
fprintf('- Camera-based lane offset correction (when camera available)\n');
fprintf('- PID steering controller for smooth lane keeping\n');
fprintf('- Speed adaptation for turns\n');
fprintf('- LED color changes per mission state\n\n');
fprintf('BEFORE RUNNING:\n');
fprintf('1. Start QLabs (Cityscape workspace)\n');
fprintf('2. Run: python Setup_Real_Scenario_fullscale_x10.py\n');
fprintf('   (keep it running - spawns QCar + traffic lights)\n');
fprintf('3. In Simulink: Press Run (Ctrl+T) or click the green Play button\n\n');
fprintf('LED BEHAVIOR:\n');
fprintf('- RED    : Initializing (5 sec) and waiting at hub\n');
fprintf('- GREEN  : Navigating (to pickup, dropoff, or hub)\n');
fprintf('- BLUE   : Stopped at pickup (passenger boarding, 2 sec)\n');
fprintf('- ORANGE : Stopped at dropoff (passenger exiting, 2 sec)\n\n');
fprintf('MISSION SEQUENCE:\n');
fprintf('  Hub [%.1f, %.1f] --(GREEN)--> Pickup [%.1f, %.1f]\n', ...
    taxi.hub_xy(1)*taxi.coord_scale, taxi.hub_xy(2)*taxi.coord_scale, ...
    taxi.pickup_xy(1)*taxi.coord_scale, taxi.pickup_xy(2)*taxi.coord_scale);
fprintf('  Pickup --(BLUE, 2s)--> Dropoff [%.1f, %.1f]\n', ...
    taxi.dropoff_xy(1)*taxi.coord_scale, taxi.dropoff_xy(2)*taxi.coord_scale);
fprintf('  Dropoff --(ORANGE, 2s)--> Hub\n\n');
fprintf('State Display: 0=INIT, 1=GO_PICKUP, 2=STOP_PICKUP,\n');
fprintf('  3=GO_DROPOFF, 4=STOP_DROPOFF, 5=RETURN_HUB, 6=WAIT_AT_HUB\n');
if ~camOk
    fprintf('\nNOTE: Camera not available - using waypoint-only mode.\n');
    fprintf('To enable camera lane keeping:\n');
    fprintf('  1. Add a Video3D Capture block (port 18942) for front camera\n');
    fprintf('  2. Add lane_detect_sfun S-Function block\n');
    fprintf('  3. Wire: Camera -> LaneDetect -> TaxiStack input 6\n');
end
fprintf('========================================\n');
