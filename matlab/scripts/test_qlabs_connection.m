%% Test QLabs Connection - ACC 2026 Self-Driving Car Competition
% Beach Autonomous Systems - CSULB
%
% This tests the connection to QLabs Virtual QCar 2.


%% ============================================================
%% Configuration
%% ============================================================

HIL_PORT = 18960;
CAMERA_FRONT_PORT = 18942;
LIDAR_PORT = 18966;
GPS_PORT = 18967;

testTimeout = 5;  % seconds

%% ============================================================
%% Check if QLabs is Running
%% ============================================================

fprintf('\n');
fprintf('========================================\n');
fprintf('  QLabs Connection Test\n');
fprintf('  ACC 2026 - Beach Autonomous Systems\n');
fprintf('========================================\n\n');

fprintf('Testing connections to QLabs Virtual QCar 2...\n\n');

%% ============================================================
%% Test HIL Port (Main Control)
%% ============================================================

fprintf('1. Testing HIL Port (%d)... ', HIL_PORT);
try
    % Try to create a TCP connection to the HIL port
    t = tcpclient('localhost', HIL_PORT, 'Timeout', testTimeout);
    clear t;
    fprintf('[OK] HIL port is accessible\n');
    hilOK = true;
catch ME
    fprintf('[FAIL] Cannot connect to HIL port\n');
    fprintf('   Error: %s\n', ME.message);
    hilOK = false;
end

%% ============================================================
%% Test Camera Port (Front)
%% ============================================================

fprintf('2. Testing Camera Front Port (%d)... ', CAMERA_FRONT_PORT);
try
    t = tcpclient('localhost', CAMERA_FRONT_PORT, 'Timeout', testTimeout);
    clear t;
    fprintf('[OK] Camera port is accessible\n');
    camOK = true;
catch ME
    fprintf('[FAIL] Cannot connect to Camera port\n');
    fprintf('   Error: %s\n', ME.message);
    camOK = false;
end

%% ============================================================
%% Test Lidar Port
%% ============================================================

fprintf('3. Testing Lidar Port (%d)... ', LIDAR_PORT);
try
    t = tcpclient('localhost', LIDAR_PORT, 'Timeout', testTimeout);
    clear t;
    fprintf('[OK] Lidar port is accessible\n');
    lidarOK = true;
catch ME
    fprintf('[FAIL] Cannot connect to Lidar port\n');
    fprintf('   Error: %s\n', ME.message);
    lidarOK = false;
end

%% ============================================================
%% Test GPS Port
%% ============================================================

fprintf('4. Testing GPS Port (%d)... ', GPS_PORT);
try
    t = tcpclient('localhost', GPS_PORT, 'Timeout', testTimeout);
    clear t;
    fprintf('[OK] GPS port is accessible\n');
    gpsOK = true;
catch ME
    fprintf('[FAIL] Cannot connect to GPS port\n');
    fprintf('   Error: %s\n', ME.message);
    gpsOK = false;
end

%% ============================================================
%% Summary
%% ============================================================

fprintf('\n');
fprintf('========================================\n');
fprintf('  Connection Test Summary\n');
fprintf('========================================\n');

allPassed = hilOK && camOK && lidarOK && gpsOK;

if allPassed
    fprintf('\n[SUCCESS] All connections working!\n');
    fprintf('\nYou can now run your Simulink model.\n');
    fprintf('The virtual QCar 2 in QLabs will respond to commands.\n');
else
    fprintf('\n[WARNING] Some connections failed.\n\n');
    fprintf('Troubleshooting:\n');
    fprintf('1. Make sure QLabs is running\n');
    fprintf('2. Make sure you selected Cityscape or Open Road workspace\n');
    fprintf('3. Wait for the workspace to fully load (buildings visible)\n');
    fprintf('\n');
    fprintf('*** MOST LIKELY CAUSE: QCar 2 is NOT spawned! ***\n');
    fprintf('Open World workspaces (Cityscape, Open Road) do NOT auto-spawn the QCar.\n');
    fprintf('Run the Python spawn script first:\n');
    fprintf('   > cd python\n');
    fprintf('   > python spawn_qcar.py\n');
    fprintf('Keep it running, then re-run this test.\n');
    fprintf('\n');
    fprintf('4. Check that no other application is using these ports\n');
    fprintf('5. Try restarting QLabs\n');
end

fprintf('\n');
fprintf('========================================\n');
fprintf('  Port Reference\n');
fprintf('========================================\n');
fprintf('HIL Control:   0@tcpip://localhost:18960\n');
fprintf('Camera Front:  0@tcpip://localhost:18942\n');
fprintf('Camera Right:  0@tcpip://localhost:18940\n');
fprintf('Camera Back:   0@tcpip://localhost:18941\n');
fprintf('Camera Left:   0@tcpip://localhost:18943\n');
fprintf('RGBD Camera:   0@tcpip://localhost:18965\n');
fprintf('Lidar:         tcpip://localhost:18966\n');
fprintf('GPS:           tcpip://localhost:18967\n');
fprintf('LED Strip:     tcpip://localhost:18969\n');
fprintf('\n');

