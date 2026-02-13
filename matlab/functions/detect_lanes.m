function [lane_center_offset, lanes] = detect_lanes(rgb_image, config)
%DETECT_LANES Detect lane markings in camera image
%
%   [lane_center_offset, lanes] = detect_lanes(rgb_image, config)
%
%   Inputs:
%       rgb_image - RGB image from camera (H x W x 3 uint8)
%       config    - Configuration struct (optional)
%                   .roi_top       - ROI top percentage (default: 0.5)
%                   .white_thresh  - White color threshold (default: 200)
%                   .yellow_h_low  - Yellow hue lower bound (default: 0.08)
%                   .yellow_h_high - Yellow hue upper bound (default: 0.20)
%                   .min_lane_pixels - Min pixels to count as lane (default: 20)
%
%   Outputs:
%       lane_center_offset - Offset from lane center (-1..+1)
%                            Negative = car left of center
%                            Positive = car right of center
%                            0 = centered or no lanes detected
%       lanes              - Struct with detected lane information
%                            .left, .right, .mask, .confidence
%
%   Example:
%       [offset, lanes] = detect_lanes(rgb);

    % Default configuration
    if nargin < 2 || isempty(config)
        config = struct();
    end
    if ~isfield(config, 'roi_top'),         config.roi_top = 0.5; end
    if ~isfield(config, 'white_thresh'),    config.white_thresh = 200; end
    if ~isfield(config, 'yellow_h_low'),    config.yellow_h_low = 0.08; end
    if ~isfield(config, 'yellow_h_high'),   config.yellow_h_high = 0.20; end
    if ~isfield(config, 'min_lane_pixels'), config.min_lane_pixels = 20; end

    [height, width, ~] = size(rgb_image);

    % Region of interest: bottom portion of image (where lanes are visible)
    roi_start = round(height * config.roi_top);
    roi = rgb_image(roi_start:end, :, :);
    [roiH, roiW, ~] = size(roi);

    % --- White lane detection (grayscale thresholding) ---
    gray = 0.299 * double(roi(:,:,1)) + 0.587 * double(roi(:,:,2)) + 0.114 * double(roi(:,:,3));
    white_mask = gray > config.white_thresh;

    % --- Yellow lane detection (HSV-based) ---
    R = double(roi(:,:,1)) / 255.0;
    G = double(roi(:,:,2)) / 255.0;
    B = double(roi(:,:,3)) / 255.0;

    maxC = max(max(R, G), B);
    minC = min(min(R, G), B);
    delta = maxC - minC;

    % Hue (0..1 scale)
    H = zeros(roiH, roiW);
    valid_delta = delta > 0.01;
    mask_rmax = (maxC == R) & valid_delta;
    mask_gmax = (maxC == G) & valid_delta & ~mask_rmax;
    mask_bmax = valid_delta & ~mask_rmax & ~mask_gmax;

    H(mask_rmax) = mod((G(mask_rmax) - B(mask_rmax)) ./ delta(mask_rmax), 6) / 6;
    H(mask_gmax) = ((B(mask_gmax) - R(mask_gmax)) ./ delta(mask_gmax) + 2) / 6;
    H(mask_bmax) = ((R(mask_bmax) - G(mask_bmax)) ./ delta(mask_bmax) + 4) / 6;
    H(H < 0) = H(H < 0) + 1;

    % Saturation
    S = zeros(roiH, roiW);
    S(maxC > 0) = delta(maxC > 0) ./ maxC(maxC > 0);

    % Yellow: specific hue range with decent saturation and brightness
    yellow_mask = H > config.yellow_h_low & H < config.yellow_h_high & ...
                  S > 0.3 & maxC > 0.3;

    % --- Combined lane mask ---
    lane_mask = white_mask | yellow_mask;

    % --- Weighted row analysis ---
    % Give more weight to pixels closer to the car (bottom of ROI)
    % Use bottom 3 horizontal bands for robustness
    bands = 3;
    bandH = max(1, floor(roiH / bands));

    left_positions = [];
    right_positions = [];
    weights = [];

    center = roiW / 2;

    for b = 1:bands
        band_start = roiH - b * bandH + 1;
        band_end = roiH - (b-1) * bandH;
        band_start = max(1, band_start);
        band_end = min(roiH, band_end);

        band_mask = lane_mask(band_start:band_end, :);
        [~, cols] = find(band_mask);

        if isempty(cols)
            continue;
        end

        % Weight: closer bands (lower b) get higher weight
        w = bands - b + 1;

        left_cols = cols(cols < center);
        right_cols = cols(cols > center);

        if numel(left_cols) >= config.min_lane_pixels
            left_positions(end+1) = median(left_cols);  %#ok<AGROW>
            weights(end+1) = w;                          %#ok<AGROW>
        end

        if numel(right_cols) >= config.min_lane_pixels
            right_positions(end+1) = median(right_cols); %#ok<AGROW>
            if numel(weights) < numel(right_positions)
                weights(end+1) = w;                      %#ok<AGROW>
            end
        end
    end

    % --- Compute left and right lane positions ---
    left_lane = [];
    right_lane = [];

    if ~isempty(left_positions)
        % Weighted average of detected positions across bands
        left_lane = mean(left_positions);
    end

    if ~isempty(right_positions)
        right_lane = mean(right_positions);
    end

    % --- Calculate offset ---
    confidence = 0;
    if ~isempty(left_lane) && ~isempty(right_lane)
        % Both lanes detected: high confidence
        lane_center = (left_lane + right_lane) / 2;
        lane_center_offset = (lane_center - center) / width;
        confidence = 1.0;
    elseif ~isempty(left_lane)
        % Only left lane: assume nominal lane width and estimate offset
        % Car is likely drifting right
        lane_center_offset = -0.15;
        confidence = 0.5;
    elseif ~isempty(right_lane)
        % Only right lane: car is likely drifting left
        lane_center_offset = 0.15;
        confidence = 0.5;
    else
        % No lanes detected
        lane_center_offset = 0;
        confidence = 0;
    end

    % Clamp
    lane_center_offset = max(-1.0, min(1.0, lane_center_offset));

    % Output struct
    lanes.left = left_lane;
    lanes.right = right_lane;
    lanes.mask = lane_mask;
    lanes.confidence = confidence;
end
