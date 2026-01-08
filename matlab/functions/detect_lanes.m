function [lane_center_offset, lanes] = detect_lanes(rgb_image, config)
%DETECT_LANES Detect lane markings in camera image
%
%   [lane_center_offset, lanes] = detect_lanes(rgb_image, config)
%
%   Inputs:
%       rgb_image - RGB image from camera
%       config    - Configuration struct (optional)
%                   .roi_top      - ROI top percentage (default: 0.5)
%                   .white_thresh - White color threshold (default: 200)
%                   .yellow_thresh - Yellow color threshold (default: [20,100,100])
%
%   Outputs:
%       lane_center_offset - Offset from lane center (negative = left, positive = right)
%       lanes              - Struct with detected lane information
%
%   Example:
%       [offset, lanes] = detect_lanes(rgb);

    % Default configuration
    if nargin < 2 || isempty(config)
        config = struct();
    end
    if ~isfield(config, 'roi_top'), config.roi_top = 0.5; end
    if ~isfield(config, 'white_thresh'), config.white_thresh = 200; end
    
    [height, width, ~] = size(rgb_image);
    
    % Define region of interest (bottom half of image)
    roi_start = round(height * config.roi_top);
    roi = rgb_image(roi_start:end, :, :);
    
    % Convert to grayscale
    gray = rgb2gray(roi);
    
    % Detect white lane markings
    white_mask = gray > config.white_thresh;
    
    % Detect yellow lane markings (convert to HSV)
    hsv = rgb2hsv(roi);
    yellow_mask = hsv(:,:,1) > 0.1 & hsv(:,:,1) < 0.2 & ...
                  hsv(:,:,2) > 0.3 & hsv(:,:,3) > 0.3;
    
    % Combined lane mask
    lane_mask = white_mask | yellow_mask;
    
    % Find lane positions
    [left_lane, right_lane] = find_lane_positions(lane_mask);
    
    % Calculate lane center offset
    image_center = width / 2;
    if ~isempty(left_lane) && ~isempty(right_lane)
        lane_center = (left_lane + right_lane) / 2;
        lane_center_offset = (lane_center - image_center) / width;
    elseif ~isempty(left_lane)
        lane_center_offset = -0.2;  % Only left lane visible, probably too far right
    elseif ~isempty(right_lane)
        lane_center_offset = 0.2;   % Only right lane visible, probably too far left
    else
        lane_center_offset = 0;     % No lanes detected
    end
    
    % Output struct
    lanes.left = left_lane;
    lanes.right = right_lane;
    lanes.mask = lane_mask;
end

function [left_lane, right_lane] = find_lane_positions(lane_mask)
%FIND_LANE_POSITIONS Find left and right lane positions
    
    [height, width] = size(lane_mask);
    left_lane = [];
    right_lane = [];
    
    % Look at bottom portion of ROI
    bottom_rows = lane_mask(round(height*0.7):end, :);
    
    % Find column indices of lane pixels
    [~, cols] = find(bottom_rows);
    
    if isempty(cols)
        return;
    end
    
    % Split into left and right based on image center
    center = width / 2;
    left_cols = cols(cols < center);
    right_cols = cols(cols > center);
    
    if ~isempty(left_cols)
        left_lane = median(left_cols);
    end
    
    if ~isempty(right_cols)
        right_lane = median(right_cols);
    end
end

