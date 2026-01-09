% QCar 2 Spawn
% -------------------------
% This will use the qvl library to spawn a car ONLY.
% 
% .. note::
% 
%     Make sure you have Quanser Interactive Labs open before running this
%     script.  This is designed to best run in QCar Cityscape.


close all;
clear all;
clc;

% --------------------------------------------------------------
% Setting MATLAB Path for the libraries
% Always keep at the start, it will make sure it finds the correct references
newPathEntry = fullfile(getenv('QAL_DIR'), 'libraries', 'matlab', 'qvl');
pathCell = regexp(path, pathsep, 'split');
if ispc  % Windows is not case-sensitive
  onPath = any(strcmpi(newPathEntry, pathCell));
else
  onPath = any(strcmp(newPathEntry, pathCell));
end

if onPath == 0
    path(path, newPathEntry)
    savepath
end
% --------------------------------------------------------------

fprintf('\n\n----------------- Communications -------------------\n\n');

qlabs = QuanserInteractiveLabs();
connection_established = qlabs.open('localhost');

if connection_established == false
    disp("Failed to open connection.")
    return
end


disp('Connected')

num_destroyed = qlabs.destroy_all_spawned_actors();

fprintf('%d actors destroyed', num_destroyed);

% Use hSystem to set the tutorial title in the upper left of the qlabs window 
%hSystem = QLabsSystem(qlabs);
%hSystem.set_title_string('QCar Tutorial')

% Initialize QLabs
hCameraQCars = QLabsFreeCamera(qlabs);
hCameraQCars.spawn_id(1, [-15.075, 26.703, 6.074], [0, 0.564, -1.586]);
hCameraQCars.possess();

disp('---QCar---');

% Spawning the QCar with radians
hQCar0 = QLabsQCar2(qlabs);
hQCar0.spawn_id(0, [-8.700, 14.643, 0.005], [0, 0, pi/2], 1);

% Spawn and destroy the existing QCar
%hQCar1 = QLabsQCar2(qlabs);
%hQCar1.spawn_id(1, [-15.075, 26.703, 6.074], [0, 0, pi/2], 1);
%hQCar1.destroy();

% Spawn a QCar with degrees
%hQCar2 = QLabsQCar2(qlabs);
%x = hQCar2.spawn_id_degrees(2, [-11.048, 14.643, 0.005], [0, 0, 90], 1);

% Pinging the QCar
%hQCar2.ping();


% % Getting images from the different cameras
% [x, camera_image] = hQCar2.get_image(hQCar2.CAMERA_CSI_FRONT);
% [x, camera_image] = hQCar2.get_image(hQCar2.CAMERA_CSI_RIGHT);
% [x, camera_image] = hQCar2.get_image(hQCar2.CAMERA_CSI_BACK);
% [x, camera_image] = hQCar2.get_image(hQCar2.CAMERA_CSI_LEFT);
% [x, camera_image] = hQCar2.get_image(hQCar2.CAMERA_RGB);
% [x, camera_image] = hQCar2.get_image(hQCar2.CAMERA_DEPTH);

%hQCar3.possess(hQCar3.CAMERA_OVERHEAD);

% Closing qlabs
qlabs.close();
disp('Done!');
