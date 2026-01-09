%% Build Complete Taxi Scenario Model (QUARC + Algorithm)
% Creates a Simulink model that:
%   1) Reads GPS position from QLabs QCar via QUARC
%   2) Runs the taxi mission FSM + steering algorithm
%   3) Writes motor throttle, steering, and LED commands via QUARC
%
% Prerequisites:
%   - QLabs running with Cityscape workspace
%   - python/Setup_Real_Scenario.py running (spawns QCar + traffic lights)
%   - QUARC toolbox installed

clear; clc;

% -------------------------------------------------------------------------
% QUARC HIL availability probe
% -------------------------------------------------------------------------
% Some QUARC installs include the Simulink blocks but do not include/enable
% the HIL board support for QCar2 (board type "qcar2"). In that case, the
% model will fail at compile time inside hil_initialize_block.
%
% To keep development moving, we probe the HIL Initialize block up-front.
% If it cannot compile with qcar2, we generate a model WITHOUT HIL blocks
% (algorithm-only mode) and print a clear warning.
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
        % Try the expected competition settings (parameter names vary by version).
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
        % Force compilation/update.
        set_param(probeModel, 'SimulationCommand', 'update');
    end

    close_system(probeModel, 0);
    % Don't leave QUARC library open (it lives under Program Files and can
    % trigger autosave permission warnings).
    try, close_system('quarc_library', 0); catch, end
catch
    useHIL = false;
    try, close_system('__taxi_quarc_hil_probe__', 0); catch, end
    try, close_system('quarc_library', 0); catch, end
end

if ~useHIL
    warning(['QCar2 QUARC HIL support not detected (HIL Initialize cannot compile with board type qcar2). ' ...
             'Building taxi_quarc in algorithm-only mode (no HIL Initialize / no HIL Write). ' ...
             'To run the full QLabs scenario via QUARC, register/install QUARC with QCar2 support.']);
end

scriptDir = fileparts(mfilename('fullpath'));
addpath(fullfile(scriptDir, '..', 'functions'));
addpath(fullfile(scriptDir, '..', 'config'));

% Load parameters
run(fullfile(scriptDir, '..', 'config', 'vehicle_params.m'));
run(fullfile(scriptDir, '..', 'config', 'taxi_scenario_params.m'));

model = 'taxi_quarc';

modelFile = fullfile(scriptDir, '..', 'models', [model '.slx']);

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
% Required for all QUARC communication
hilInitPath = [model '/HIL_Initialize'];
hilInitOk = false;
if useHIL
    try
        add_block('quarc_library/Data Acquisition/Generic/Configuration/HIL Initialize', hilInitPath, ...
            'Position', [x0 y0 x0+150 y0+60]);
        hilInitOk = true;
    catch
        % Try alternate library path
        try
            add_block('quarc_library/HIL Initialize', hilInitPath, ...
                'Position', [x0 y0 x0+150 y0+60]);
            hilInitOk = true;
        catch
        end
    end

    if hilInitOk
        % Try various parameter name variants for board type and identifier
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
        warning('Could not add HIL Initialize block. QUARC may not be installed. Adding placeholder.');
        add_block('simulink/Sources/Constant', hilInitPath, 'Position', [x0 y0 x0+80 y0+40], 'Value', '0');
    end
else
    add_block('simulink/Sources/Constant', hilInitPath, 'Position', [x0 y0 x0+80 y0+40], 'Value', '0');
end

% Terminate HIL Initialize output (some QUARC versions expose an output port
% and Simulink warns if it's left unconnected).
try
    termPath = [model '/HIL_Init_Term'];
    add_block('simulink/Sinks/Terminator', termPath, 'Position', [x0+170 y0+15 x0+190 y0+35]);
    add_line(model, 'HIL_Initialize/1', 'HIL_Init_Term/1');
catch
end

%% ==================== GPS Read ====================
% GPS provides position [x, y, z] and orientation [roll, pitch, yaw]
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
    % Try to set URI
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
    % Simulated GPS: start at hub position [x, y, z, roll, pitch, yaw]
    add_block('simulink/Sources/Constant', gpsPath, 'Position', [x0 y0+2*dy x0+100 y0+2*dy+40]);
    % Match coordinate scaling used by the scenario (e.g., fullscale x10).
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
% Selector for pos_xy (elements 1,2)
posSelPath = [model '/Pos_XY_Selector'];
add_block('simulink/Signal Routing/Selector', posSelPath, 'Position', [x0+dx y0+2*dy x0+dx+60 y0+2*dy+40]);
set_param(posSelPath, 'NumberOfDimensions', '1');
set_param(posSelPath, 'IndexMode', 'One-based');
set_param(posSelPath, 'InputPortWidth', '6');
set_param(posSelPath, 'IndexOptions', 'Index vector (dialog)');
set_param(posSelPath, 'Indices', '[1 2]');
set_param(posSelPath, 'OutputSizes', '2');

% Selector for heading (element 6 = yaw)
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
% Derivative of position magnitude for speed estimate (simplified)
% In practice you'd use wheel encoders or differentiate GPS more carefully
add_block('simulink/Sources/Constant', [model '/Speed_Est'], 'Position', [x0 y0+3*dy x0+80 y0+3*dy+30], 'Value', '0.1');

%% ==================== Taxi Stack S-Function ====================
sfunPath = [model '/TaxiStack'];
pos = [x0+2*dx y0+2*dy x0+2*dx+200 y0+2*dy+140];

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
        % Try to set channel parameters (names vary by QUARC version)
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

%% ==================== Mux for commands ====================
add_block('simulink/Signal Routing/Mux', [model '/Cmd_Mux'], 'Position', [x0+3.5*dx y0+2*dy+20 x0+3.5*dx+10 y0+2*dy+70]);
set_param([model '/Cmd_Mux'], 'Inputs', '2');

%% ==================== Wiring ====================
% GPS -> Selectors
add_line(model, 'GPS_Read/1', 'Pos_XY_Selector/1');
add_line(model, 'GPS_Read/1', 'Heading_Selector/1');

% Inputs -> TaxiStack
add_line(model, 'Pos_XY_Selector/1', 'TaxiStack/1');
add_line(model, 'Heading_Selector/1', 'TaxiStack/2');
add_line(model, 'Speed_Est/1', 'TaxiStack/3');
add_line(model, 'Clock/1', 'TaxiStack/4');
add_line(model, 'Reset/1', 'TaxiStack/5');

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
fprintf('BEFORE RUNNING:\n');
fprintf('1. Start QLabs (Cityscape workspace)\n');
fprintf('2. Run: python Setup_Real_Scenario.py\n');
fprintf('   (keep it running - spawns QCar + traffic lights)\n');
fprintf('3. In Simulink: Press Run (Ctrl+T) or click the green Play button\n\n');
fprintf('EXPECTED BEHAVIOR:\n');
fprintf('- QCar starts at Taxi Hub with RED LED\n');
fprintf('- Drives to pickup [0.125, 4.395] with GREEN LED\n');
fprintf('- Stops, LED turns BLUE (passenger pickup)\n');
fprintf('- Drives to dropoff [-0.905, 0.800] with GREEN LED\n');
fprintf('- Stops, LED turns ORANGE (passenger dropoff)\n');
fprintf('- Returns to Taxi Hub, LED turns RED\n\n');
fprintf('State Display values:\n');
fprintf('  0=INIT, 1=GO_PICKUP, 2=STOP_PICKUP,\n');
fprintf('  3=GO_DROPOFF, 4=STOP_DROPOFF, 5=RETURN_HUB, 6=WAIT_AT_HUB\n');
fprintf('========================================\n');
