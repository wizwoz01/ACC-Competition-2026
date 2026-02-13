function lane_detect_sfun(block)
% Level-2 MATLAB S-Function for camera-based lane detection.
%
% Processes an RGB camera image and outputs the lane center offset.
% Designed to be wired between a QUARC Video3D Capture block and
% the TaxiStack S-function's lane_offset input.
%
% Input:
%   1) rgb_image [N x 1] uint8 - Flattened RGB image (H*W*3 elements)
%      Reshape internally to [H x W x 3] using configured dimensions.
%
% Output:
%   1) lane_offset [1x1] double - Lane center offset (-1..+1, 0 = centered)
%      Positive = car is right of center, Negative = car is left of center
%
% Configuration:
%   Image dimensions are set as dialog parameters or use defaults.
%   Default: 820 x 410 (QCar2 front camera resolution).

setup(block);

function setup(block)
    block.NumDialogPrms = 0;

    block.NumInputPorts  = 1;
    block.NumOutputPorts = 1;

    % Input: flattened RGB image as uint8
    % For QCar2 front camera: 820 * 410 * 3 = 1008600 elements
    % Use -1 for dynamically-sized input (accommodates different cameras)
    block.InputPort(1).Dimensions = -1;
    block.InputPort(1).DatatypeID = 3;    % uint8
    block.InputPort(1).Complexity = 'Real';
    block.InputPort(1).DirectFeedthrough = true;

    % Output: lane_offset [1x1] double
    block.OutputPort(1).Dimensions = 1;
    block.OutputPort(1).DatatypeID = 0;   % double
    block.OutputPort(1).Complexity = 'Real';

    block.SampleTimes = [-1 0];

    block.RegBlockMethod('Outputs', @Outputs);
    block.RegBlockMethod('SetInputPortDimensions', @SetInputPortDimensions);
end

function SetInputPortDimensions(block, idx, di)
    block.InputPort(idx).Dimensions = di;
end

function Outputs(block)
    persistent prevOffset  % smoothing
    persistent camConfig

    if isempty(prevOffset)
        prevOffset = 0.0;
    end

    if isempty(camConfig)
        camConfig = struct();
        camConfig.width = 820;
        camConfig.height = 410;
        camConfig.roi_top = 0.5;
        camConfig.white_thresh = 200;
        % Try to load from workspace
        try
            if evalin('base', "exist('sensors','var')")
                s = evalin('base', 'sensors');
                if isfield(s, 'camera')
                    if isfield(s.camera, 'width'),  camConfig.width = s.camera.width; end
                    if isfield(s.camera, 'height'), camConfig.height = s.camera.height; end
                end
            end
            if evalin('base', "exist('lanes','var')")
                l = evalin('base', 'lanes');
                if isfield(l, 'roi_top'),      camConfig.roi_top = l.roi_top; end
                if isfield(l, 'white_thresh'),  camConfig.white_thresh = l.white_thresh; end
            end
        catch
        end
    end

    % Read input
    raw = block.InputPort(1).Data;
    imgW = camConfig.width;
    imgH = camConfig.height;
    expectedSize = imgW * imgH * 3;

    % Check if input matches expected camera dimensions
    if numel(raw) ~= expectedSize
        % Wrong size or no camera connected - output previous/zero offset
        block.OutputPort(1).Data = prevOffset * 0.9;  % decay toward zero
        prevOffset = prevOffset * 0.9;
        return;
    end

    % Reshape to H x W x 3 RGB image
    try
        rgb_image = reshape(uint8(raw), [imgH, imgW, 3]);
    catch
        block.OutputPort(1).Data = 0.0;
        return;
    end

    % Run lane detection
    lane_offset = detect_lane_offset(rgb_image, camConfig);

    % Exponential smoothing to reduce jitter
    alpha = 0.3;  % smoothing factor (0 = no update, 1 = no smoothing)
    if isfinite(lane_offset) && lane_offset ~= 0
        smoothed = alpha * lane_offset + (1 - alpha) * prevOffset;
    else
        % No lanes detected - slowly decay to zero
        smoothed = prevOffset * 0.95;
    end

    prevOffset = smoothed;
    block.OutputPort(1).Data = smoothed;
end

%% =====================================================================
%  LANE DETECTION ALGORITHM
%  =====================================================================
function lane_offset = detect_lane_offset(rgb_image, config)
% Detect lane center offset from an RGB camera image.
%
% Returns offset in range [-1, +1]:
%   Negative = car is left of lane center
%   Positive = car is right of lane center
%   0 = centered (or no lanes detected)

    [height, width, ~] = size(rgb_image);

    % Region of interest: bottom portion of image
    roi_start = round(height * config.roi_top);
    roi = rgb_image(roi_start:end, :, :);
    [roiH, roiW, ~] = size(roi);

    % Convert to grayscale for white lane detection
    gray = 0.299 * double(roi(:,:,1)) + 0.587 * double(roi(:,:,2)) + 0.114 * double(roi(:,:,3));

    % White lane detection: bright pixels
    white_mask = gray > config.white_thresh;

    % Yellow lane detection: HSV-based
    % Convert ROI to HSV manually (avoids dependency on rgb2hsv)
    R = double(roi(:,:,1)) / 255.0;
    G = double(roi(:,:,2)) / 255.0;
    B = double(roi(:,:,3)) / 255.0;

    maxC = max(max(R, G), B);
    minC = min(min(R, G), B);
    delta = maxC - minC;

    % Hue calculation (0..1 scale)
    H = zeros(roiH, roiW);
    mask_rmax = (maxC == R) & (delta > 0);
    mask_gmax = (maxC == G) & (delta > 0);
    mask_bmax = (maxC == B) & (delta > 0);

    H(mask_rmax) = mod((G(mask_rmax) - B(mask_rmax)) ./ delta(mask_rmax), 6) / 6;
    H(mask_gmax) = ((B(mask_gmax) - R(mask_gmax)) ./ delta(mask_gmax) + 2) / 6;
    H(mask_bmax) = ((R(mask_bmax) - G(mask_bmax)) ./ delta(mask_bmax) + 4) / 6;
    H(H < 0) = H(H < 0) + 1;

    % Saturation
    S = zeros(roiH, roiW);
    S(maxC > 0) = delta(maxC > 0) ./ maxC(maxC > 0);

    % Value
    V = maxC;

    % Yellow: Hue roughly 0.08-0.20 (30-72 degrees), high saturation, high value
    yellow_mask = H > 0.08 & H < 0.20 & S > 0.3 & V > 0.3;

    % Combined lane mask
    lane_mask = white_mask | yellow_mask;

    % Focus on bottom portion of ROI for more reliable detection
    bottomStart = round(roiH * 0.5);
    bottom_mask = lane_mask(bottomStart:end, :);
    [~, bW] = size(bottom_mask);

    % Find lane pixel columns
    [~, cols] = find(bottom_mask);

    if isempty(cols)
        lane_offset = 0;
        return;
    end

    % Split into left and right lanes
    center = bW / 2;
    left_cols = cols(cols < center);
    right_cols = cols(cols > center);

    % Calculate lane center
    if ~isempty(left_cols) && ~isempty(right_cols)
        left_pos = median(left_cols);
        right_pos = median(right_cols);
        lane_center = (left_pos + right_pos) / 2;
        lane_offset = (lane_center - center) / width;
    elseif ~isempty(left_cols)
        % Only left lane visible: car is probably drifting right
        lane_offset = -0.15;
    elseif ~isempty(right_cols)
        % Only right lane visible: car is probably drifting left
        lane_offset = 0.15;
    else
        lane_offset = 0;
    end

    % Clamp
    lane_offset = max(-1.0, min(1.0, lane_offset));
end

end
