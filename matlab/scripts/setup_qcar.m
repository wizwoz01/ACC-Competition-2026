%% Setup QCar 2 - ACC 2026 Self-Driving Car Competition
% Beach Autonomous Systems - CSULB
%
% This script launches QLabs and prepares for QCar 2 development.
% starting point for each development session.

%% ============================================================
%% STEP 1: Clear Environment
%% ============================================================

clear; clc; close all;

fprintf('\n');
fprintf('========================================\n');
fprintf('  QCar 2 Development Setup\n');
fprintf('  ACC 2026 - Beach Autonomous Systems\n');
fprintf('========================================\n\n');

%% ============================================================
%% STEP 2: Add Project Paths
%% ============================================================

fprintf('Adding project paths...\n');

% Get the script directory
scriptDir = fileparts(mfilename('fullpath'));

% Add function paths
addpath(fullfile(scriptDir, '..', 'functions'));
addpath(fullfile(scriptDir, '..', 'config'));

fprintf('  - Functions path added\n');
fprintf('  - Config path added\n');

%% ============================================================
%% STEP 3: Check QUARC Installation
%% ============================================================

fprintf('\nChecking QUARC installation...\n');

try
    % Check if QUARC library exists
    if exist('quarc_library', 'file')
        fprintf('  [OK] QUARC library found\n');
        quarcOK = true;
    else
        fprintf('  [WARNING] QUARC library not found in path\n');
        fprintf('  You may need to run: qc_open_library(''quarc_library'')\n');
        quarcOK = false;
    end
catch
    fprintf('  [WARNING] Could not verify QUARC installation\n');
    quarcOK = false;
end

%% ============================================================
%% STEP 4: Check QLabs Add-on
%% ============================================================

fprintf('\nChecking QLabs add-on...\n');

if exist('QLabs', 'class')
    fprintf('  [OK] QLabs add-on found\n');
    qlabsOK = true;
else
    fprintf('  [WARNING] QLabs add-on not found\n');
    fprintf('  Install from: Add-Ons -> Get Add-Ons -> "Quanser Interactive Labs for MATLAB"\n');
    qlabsOK = false;
end

%% ============================================================
%% STEP 5: Launch QLabs
%% ============================================================

if qlabsOK
    fprintf('\n');
    response = input('Launch QLabs now? (y/n): ', 's');
    
    if strcmpi(response, 'y')
        fprintf('Launching QLabs...\n');
        fprintf('(This may take a moment)\n\n');
        
        try
            QLabs.launch;
            fprintf('[OK] QLabs launched!\n\n');
            
            fprintf('========================================\n');
            fprintf('  NEXT STEPS (Manual Actions Required)\n');
            fprintf('========================================\n');
            fprintf('1. Log in to QLabs with your Quanser credentials\n');
            fprintf('2. Select workspace: Cityscape or Open Road\n');
            fprintf('3. Wait for the environment to fully load\n');
            fprintf('\n');
            fprintf('*** IMPORTANT: Open World workspaces do NOT auto-spawn QCar ***\n');
            fprintf('4. Run the Python spawn script to create the QCar 2:\n');
            fprintf('   > cd python\n');
            fprintf('   > python spawn_qcar.py\n');
            fprintf('   (Keep the Python script running while using Simulink)\n');
            fprintf('\n');
            fprintf('5. After QCar is spawned, test connection:\n');
            fprintf('   >> run test_qlabs_connection\n');
            fprintf('\n');
            fprintf('6. Then run your Simulink model to control the vehicle.\n');
            fprintf('\n');
        catch ME
            fprintf('[ERROR] Failed to launch QLabs: %s\n', ME.message);
        end
    else
        fprintf('Skipping QLabs launch.\n');
    end
else
    fprintf('\n[ERROR] Cannot launch QLabs - add-on not installed.\n');
end

%% ============================================================
%% STEP 6: Display Configuration Reference
%% ============================================================

fprintf('\n');
fprintf('========================================\n');
fprintf('  QCar 2 Configuration Reference\n');
fprintf('========================================\n\n');

fprintf('HIL Initialize Block Settings:\n');
fprintf('  Board type:       qcar2\n');
fprintf('  Board identifier: 0@tcpip://localhost:18960\n');
fprintf('  Board name:       QCar2 \n');
fprintf('\n');

fprintf('Port Reference Table:\n');
fprintf('  %-20s %-10s %s\n', 'Component', 'Port', 'URI');
fprintf('  %-20s %-10s %s\n', '--------', '----', '---');
fprintf('  %-20s %-10d %s\n', 'HIL Control', 18960, '0@tcpip://localhost:18960');
fprintf('  %-20s %-10d %s\n', 'Camera Front', 18942, '0@tcpip://localhost:18942');
fprintf('  %-20s %-10d %s\n', 'Camera Right', 18940, '0@tcpip://localhost:18940');
fprintf('  %-20s %-10d %s\n', 'Camera Back', 18941, '0@tcpip://localhost:18941');
fprintf('  %-20s %-10d %s\n', 'Camera Left', 18943, '0@tcpip://localhost:18943');
fprintf('  %-20s %-10d %s\n', 'RGBD Camera', 18965, '0@tcpip://localhost:18965');
fprintf('  %-20s %-10d %s\n', 'Lidar', 18966, 'tcpip://localhost:18966');
fprintf('  %-20s %-10d %s\n', 'GPS', 18967, 'tcpip://localhost:18967');
fprintf('  %-20s %-10d %s\n', 'LED Strip', 18969, 'tcpip://localhost:18969');
fprintf('\n');

fprintf('========================================\n');
fprintf('  Setup Complete\n');
fprintf('========================================\n\n');
