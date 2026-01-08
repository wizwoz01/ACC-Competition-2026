%% Test Control - ACC 2026 Self-Driving Car Competition
% Beach Autonomous Systems - CSULB
%
% Tests basic vehicle control commands.

%% Ensure QCar is initialized
if ~exist('qcar', 'var')
    error('QCar not initialized. Run setup_qcar.m first.');
end

%% Test Parameters
test_speed = 0.3;       % m/s
test_duration = 2;      % seconds

%% Test 1: Drive Forward
fprintf('Test 1: Driving forward...\n');
qcar.write_velocity(test_speed, 0);
pause(test_duration);
qcar.write_velocity(0, 0);
fprintf('Stopped.\n');
pause(1);

%% Test 2: Turn Left
fprintf('Test 2: Turning left...\n');
qcar.write_velocity(test_speed, 0.3);
pause(test_duration);
qcar.write_velocity(0, 0);
fprintf('Stopped.\n');
pause(1);

%% Test 3: Turn Right
fprintf('Test 3: Turning right...\n');
qcar.write_velocity(test_speed, -0.3);
pause(test_duration);
qcar.write_velocity(0, 0);
fprintf('Stopped.\n');
pause(1);

%% Test 4: Reverse
fprintf('Test 4: Reversing...\n');
qcar.write_velocity(-test_speed, 0);
pause(test_duration);
qcar.write_velocity(0, 0);
fprintf('Stopped.\n');

%% Summary
fprintf('\n=== Control Test Complete ===\n');
fprintf('All control commands executed successfully.\n');

