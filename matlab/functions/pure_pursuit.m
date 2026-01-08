function steering_angle = pure_pursuit(current_pos, current_heading, waypoints, lookahead_distance, wheelbase)
%PURE_PURSUIT Calculate steering angle using pure pursuit algorithm
%
%   steering_angle = pure_pursuit(current_pos, current_heading, waypoints, lookahead_distance, wheelbase)
%
%   Inputs:
%       current_pos        - Current vehicle position [x, y]
%       current_heading    - Current vehicle heading (radians)
%       waypoints          - Nx2 matrix of waypoints [[x1,y1]; [x2,y2]; ...]
%       lookahead_distance - Lookahead distance (meters)
%       wheelbase          - Vehicle wheelbase (meters), default 0.256 for QCar2
%
%   Outputs:
%       steering_angle - Steering angle (radians)
%
%   Example:
%       waypoints = [[0,0]; [1,0]; [2,0.5]; [3,1]];
%       steering = pure_pursuit([0,0], 0, waypoints, 0.5, 0.256);

    if nargin < 5
        wheelbase = 0.256;  % QCar 2 wheelbase
    end
    
    % Find the lookahead point
    lookahead_point = find_lookahead_point(current_pos, waypoints, lookahead_distance);
    
    if isempty(lookahead_point)
        steering_angle = 0;
        return;
    end
    
    % Transform lookahead point to vehicle frame
    dx = lookahead_point(1) - current_pos(1);
    dy = lookahead_point(2) - current_pos(2);
    
    % Rotate to vehicle frame
    local_x = dx * cos(-current_heading) - dy * sin(-current_heading);
    local_y = dx * sin(-current_heading) + dy * cos(-current_heading);
    
    % Calculate curvature
    L = sqrt(local_x^2 + local_y^2);
    if L < 0.001
        steering_angle = 0;
        return;
    end
    
    curvature = 2 * local_y / (L^2);
    
    % Calculate steering angle
    steering_angle = atan(wheelbase * curvature);
    
    % Clamp steering angle
    max_steering = 0.5;  % radians
    steering_angle = max(-max_steering, min(max_steering, steering_angle));
end

function lookahead_point = find_lookahead_point(current_pos, waypoints, lookahead_distance)
%FIND_LOOKAHEAD_POINT Find the lookahead point on the path
    
    lookahead_point = [];
    n = size(waypoints, 1);
    
    for i = 1:n-1
        p1 = waypoints(i, :);
        p2 = waypoints(i+1, :);
        
        % Find intersection with circle of radius lookahead_distance
        intersections = line_circle_intersection(p1, p2, current_pos, lookahead_distance);
        
        if ~isempty(intersections)
            % Return the furthest intersection along the path
            for j = 1:size(intersections, 1)
                pt = intersections(j, :);
                % Check if point is on the line segment
                if is_on_segment(p1, p2, pt)
                    lookahead_point = pt;
                end
            end
        end
    end
    
    % If no intersection found, use the last waypoint
    if isempty(lookahead_point) && n > 0
        lookahead_point = waypoints(end, :);
    end
end

function intersections = line_circle_intersection(p1, p2, center, radius)
%LINE_CIRCLE_INTERSECTION Find intersections between line segment and circle
    
    d = p2 - p1;
    f = p1 - center;
    
    a = dot(d, d);
    b = 2 * dot(f, d);
    c = dot(f, f) - radius^2;
    
    discriminant = b^2 - 4*a*c;
    
    intersections = [];
    
    if discriminant >= 0
        discriminant = sqrt(discriminant);
        t1 = (-b - discriminant) / (2*a);
        t2 = (-b + discriminant) / (2*a);
        
        if t1 >= 0 && t1 <= 1
            intersections = [intersections; p1 + t1 * d];
        end
        if t2 >= 0 && t2 <= 1 && abs(t2 - t1) > 0.001
            intersections = [intersections; p1 + t2 * d];
        end
    end
end

function result = is_on_segment(p1, p2, pt)
%IS_ON_SEGMENT Check if point is on line segment
    
    min_x = min(p1(1), p2(1)) - 0.001;
    max_x = max(p1(1), p2(1)) + 0.001;
    min_y = min(p1(2), p2(2)) - 0.001;
    max_y = max(p1(2), p2(2)) + 0.001;
    
    result = pt(1) >= min_x && pt(1) <= max_x && pt(2) >= min_y && pt(2) <= max_y;
end

