function [tl_state, stopDist, stopRequired] = pyDetectWrapper(rgb, depth)
% This is executed by MATLAB (interpreted), not codegen.
% It can freely call Python.

persistent vision initialized

% Defaults (must always return 3 outputs)
tl_state = uint8(0);
stopDist = inf;
stopRequired = false;

if isempty(initialized)
    initialized = false;
end

try
    if ~initialized
        % Ensure this folder is on Python import path
        p = pwd;
        if ~any(strcmp(cell(py.sys.path), p))
            py.sys.path.insert(int32(0), p);
        end

        % Import your python module and create the vision object once
        py.importlib.import_module('qcar_vision');
        vision = py.qcar_vision.QCarVision();
        initialized = true;
    end

    % Convert MATLAB arrays to numpy
    py_rgb = py.numpy.array(rgb);
    py_depth = py.numpy.array(depth);

    % Call python (expects (int, float, bool))
    out = vision.step(py_rgb, py_depth);

    tl_state = uint8(out{1});
    stopDist = double(out{2});
    stopRequired = logical(out{3});

catch
    % If anything fails, return safe defaults
    tl_state = uint8(0);
    stopDist = inf;
    stopRequired = false;
end
end