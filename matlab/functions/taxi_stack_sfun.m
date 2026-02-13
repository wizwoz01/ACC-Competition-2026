function taxi_stack_sfun(block)
% Level-2 MATLAB S-Function: Taxi mission FSM + waypoint pure pursuit + lane keeping.
%
% Enhanced version with:
%   - Pure pursuit waypoint following (uses paths from waypoints.txt)
%   - Camera-based lane offset correction
%   - PID steering controller
%   - Speed adaptation for turns
%   - Same FSM states and LED behavior as before
%
% Inputs:
%   1) pos_xy      [2x1] double  - GPS position (x,y) in scaled coordinates
%   2) heading_rad [1x1] double  - vehicle heading (radians)
%   3) speed_mps   [1x1] double  - speed estimate (m/s)
%   4) t           [1x1] double  - simulation clock (seconds)
%   5) reset       [1x1] double  - reset flag (nonzero to restart)
%   6) lane_offset [1x1] double  - lane center offset from camera (-1..+1, 0=centered)
%
% Outputs:
%   1) v_cmd_mps     [1x1] double  - speed command
%   2) delta_cmd_rad [1x1] double  - steering command
%   3) led_rgb       [3x1] uint8   - LED color [R;G;B]
%   4) state         [1x1] uint8   - mission state id

setup(block);

function setup(block)
    block.NumDialogPrms = 0;

    block.NumInputPorts  = 6;
    block.NumOutputPorts = 4;

    % Input 1: pos_xy [2x1]
    block.InputPort(1).Dimensions = 2;
    block.InputPort(1).DatatypeID = 0;
    block.InputPort(1).Complexity = 'Real';
    block.InputPort(1).DirectFeedthrough = true;

    % Input 2: heading_rad [1x1]
    block.InputPort(2).Dimensions = 1;
    block.InputPort(2).DatatypeID = 0;
    block.InputPort(2).Complexity = 'Real';
    block.InputPort(2).DirectFeedthrough = true;

    % Input 3: speed_mps [1x1]
    block.InputPort(3).Dimensions = 1;
    block.InputPort(3).DatatypeID = 0;
    block.InputPort(3).Complexity = 'Real';
    block.InputPort(3).DirectFeedthrough = true;

    % Input 4: t [1x1]
    block.InputPort(4).Dimensions = 1;
    block.InputPort(4).DatatypeID = 0;
    block.InputPort(4).Complexity = 'Real';
    block.InputPort(4).DirectFeedthrough = true;

    % Input 5: reset [1x1]
    block.InputPort(5).Dimensions = 1;
    block.InputPort(5).DatatypeID = 0;
    block.InputPort(5).Complexity = 'Real';
    block.InputPort(5).DirectFeedthrough = true;

    % Input 6: lane_offset [1x1]
    block.InputPort(6).Dimensions = 1;
    block.InputPort(6).DatatypeID = 0;
    block.InputPort(6).Complexity = 'Real';
    block.InputPort(6).DirectFeedthrough = true;

    % Output 1: v_cmd_mps [1x1] double
    block.OutputPort(1).Dimensions = 1;
    block.OutputPort(1).DatatypeID = 0;
    block.OutputPort(1).Complexity = 'Real';

    % Output 2: delta_cmd_rad [1x1] double
    block.OutputPort(2).Dimensions = 1;
    block.OutputPort(2).DatatypeID = 0;
    block.OutputPort(2).Complexity = 'Real';

    % Output 3: led_rgb [3x1] uint8
    block.OutputPort(3).Dimensions = 3;
    block.OutputPort(3).DatatypeID = 3;
    block.OutputPort(3).Complexity = 'Real';

    % Output 4: state [1x1] uint8
    block.OutputPort(4).Dimensions = 1;
    block.OutputPort(4).DatatypeID = 3;
    block.OutputPort(4).Complexity = 'Real';

    block.SampleTimes = [-1 0];
    block.RegBlockMethod('Outputs', @Outputs);
end

function Outputs(block)
    %% Persistent state
    persistent mode stopStartTime initStartTime
    persistent taxiCfg
    persistent wp_pickup wp_dropoff wp_hub    % waypoint paths
    persistent wpIdx                          % current waypoint index
    persistent steer_integral steer_prev_err  % PID state
    persistent prev_t                         % for dt calculation

    %% ===================== Taxi Config =====================
    defaultTaxi = struct();
    defaultTaxi.coord_scale = 10.0;
    defaultTaxi.hub_xy     = [-1.205; -0.830];
    defaultTaxi.pickup_xy  = [ 0.125;  4.395];
    defaultTaxi.dropoff_xy = [-0.905;  0.800];
    defaultTaxi.arrival_radius_m = 0.30;
    defaultTaxi.stop_hold_sec = 2.0;
    defaultTaxi.init_hold_sec = 5.0;
    defaultTaxi.cruise_speed_mps = 0.50;
    defaultTaxi.stop_speed_thresh_mps = 0.05;
    defaultTaxi.led_red    = uint8([255; 0; 0]);
    defaultTaxi.led_green  = uint8([0; 255; 0]);
    defaultTaxi.led_blue   = uint8([0; 0; 255]);
    defaultTaxi.led_orange = uint8([255; 165; 0]);

    if isempty(taxiCfg)
        taxiCfg = defaultTaxi;
        try
            if evalin('base', "exist('taxi','var')")
                wsTaxi = evalin('base', 'taxi');
                if isstruct(wsTaxi)
                    taxiCfg = wsTaxi;
                end
            end
        catch
            taxiCfg = defaultTaxi;
        end
    end

    %% ===================== Control Parameters =====================
    % Pure pursuit
    WHEELBASE = 0.256;          % QCar2 wheelbase (m)
    LOOKAHEAD = 3.0;            % Lookahead distance in scaled coords
    MIN_LOOKAHEAD = 1.5;
    MAX_LOOKAHEAD = 5.0;

    % PID gains for steering
    KP_STEER = 1.2;
    KI_STEER = 0.05;
    KD_STEER = 0.15;

    % Lane offset blending gain
    K_LANE = 0.6;

    % Speed control
    SPEED_TURN_FACTOR = 0.4;   % Slow down to this fraction in sharp turns
    STEER_THRESH_SLOW = 0.2;   % Steering angle above which we slow down

    MAX_STEER = 0.5;           % Max steering angle (rad)

    %% ===================== Coordinate Scaling =====================
    coord_scale = 1.0;
    try, coord_scale = double(taxiCfg.coord_scale); catch, end
    if ~isfinite(coord_scale) || coord_scale <= 0
        coord_scale = 1.0;
    end

    hub_xy     = taxiCfg.hub_xy(:)     * coord_scale;
    pickup_xy  = taxiCfg.pickup_xy(:)  * coord_scale;
    dropoff_xy = taxiCfg.dropoff_xy(:) * coord_scale;

    arrival_radius      = double(taxiCfg.arrival_radius_m) * coord_scale;
    stop_hold_sec       = double(taxiCfg.stop_hold_sec);
    init_hold_sec       = double(taxiCfg.init_hold_sec);
    cruise_speed_mps    = double(taxiCfg.cruise_speed_mps);
    stop_speed_thresh   = double(taxiCfg.stop_speed_thresh_mps);

    led_red    = uint8(taxiCfg.led_red(:));
    led_green  = uint8(taxiCfg.led_green(:));
    led_blue   = uint8(taxiCfg.led_blue(:));
    led_orange = uint8(taxiCfg.led_orange(:));

    %% ===================== Load Waypoints (once) =====================
    if isempty(wp_pickup)
        wp_pickup = get_waypoints_pickup();
        wp_dropoff = get_waypoints_dropoff();
        wp_hub = get_waypoints_hub();
    end

    %% ===================== State IDs =====================
    STATE_INIT         = uint8(0);
    STATE_GO_PICKUP    = uint8(1);
    STATE_STOP_PICKUP  = uint8(2);
    STATE_GO_DROPOFF   = uint8(3);
    STATE_STOP_DROPOFF = uint8(4);
    STATE_RETURN_HUB   = uint8(5);
    STATE_WAIT_AT_HUB  = uint8(6);

    %% ===================== Read Inputs =====================
    pos_xy      = block.InputPort(1).Data;
    heading_rad = block.InputPort(2).Data;
    speed_mps   = block.InputPort(3).Data;
    t           = block.InputPort(4).Data;
    reset       = block.InputPort(5).Data;
    lane_offset = block.InputPort(6).Data;

    %% ===================== Initialize =====================
    if isempty(mode) || (~isempty(reset) && reset(1) ~= 0)
        mode = STATE_INIT;
        stopStartTime = -1.0;
        wpIdx = 1;
        steer_integral = 0.0;
        steer_prev_err = 0.0;
        if isfinite(t)
            initStartTime = t;
        else
            initStartTime = 0.0;
        end
        prev_t = t;
    end

    if isempty(steer_integral), steer_integral = 0.0; end
    if isempty(steer_prev_err), steer_prev_err = 0.0; end
    if isempty(wpIdx), wpIdx = 1; end
    if isempty(prev_t), prev_t = t; end

    %% ===================== Compute dt =====================
    dt = t - prev_t;
    if ~isfinite(dt) || dt <= 0
        dt = 0.05;  % fallback to 20Hz
    end
    prev_t = t;

    %% ===================== Default Outputs =====================
    target_xy = hub_xy;
    v_cmd_mps = 0.0;
    led_rgb = led_red;

    %% ===================== Validate Inputs =====================
    pos_xy = pos_xy(:);
    if numel(pos_xy) ~= 2 || any(~isfinite(pos_xy))
        mode = STATE_WAIT_AT_HUB;
        target_xy = hub_xy;
        v_cmd_mps = 0.0;
        led_rgb = led_red;
        stopStartTime = -1.0;
        block.OutputPort(1).Data = 0.0;
        block.OutputPort(2).Data = 0.0;
        block.OutputPort(3).Data = led_red;
        block.OutputPort(4).Data = mode;
        return;
    end

    if ~isfinite(heading_rad), heading_rad = 0; end
    if ~isfinite(lane_offset), lane_offset = 0; end
    lane_offset = max(-1.0, min(1.0, lane_offset));

    %% ===================== Arrival Checks =====================
    dxp = pos_xy(1) - pickup_xy(1);  dyp = pos_xy(2) - pickup_xy(2);
    dxd = pos_xy(1) - dropoff_xy(1); dyd = pos_xy(2) - dropoff_xy(2);
    dxh = pos_xy(1) - hub_xy(1);     dyh = pos_xy(2) - hub_xy(2);

    arrived_pickup  = sqrt(dxp*dxp + dyp*dyp) <= arrival_radius;
    arrived_dropoff = sqrt(dxd*dxd + dyd*dyd) <= arrival_radius;
    arrived_hub     = sqrt(dxh*dxh + dyh*dyh) <= arrival_radius;

    isStopped = true;
    if isfinite(speed_mps)
        isStopped = speed_mps <= stop_speed_thresh;
    end

    %% ===================== Select Current Waypoints =====================
    % Will be set per state; default empty
    active_waypoints = [];
    use_waypoints = false;

    %% ===================== FSM (same states, same LED) =====================
    stopTimerDone = false;
    if stopStartTime >= 0
        stopTimerDone = (t - stopStartTime) >= stop_hold_sec;
    end

    switch mode
        case STATE_INIT
            target_xy = hub_xy;
            v_cmd_mps = 0.0;
            led_rgb = led_red;
            stopStartTime = -1.0;
            if isfinite(t) && (t - initStartTime) >= init_hold_sec
                mode = STATE_GO_PICKUP;
                wpIdx = 1;  % reset waypoint index for new path
                steer_integral = 0.0;
                steer_prev_err = 0.0;
            end

        case STATE_GO_PICKUP
            target_xy = pickup_xy;
            v_cmd_mps = cruise_speed_mps;
            led_rgb = led_green;
            stopStartTime = -1.0;
            active_waypoints = wp_pickup;
            use_waypoints = true;
            if arrived_pickup
                mode = STATE_STOP_PICKUP;
                stopStartTime = -1.0;
                wpIdx = 1;
                steer_integral = 0.0;
                steer_prev_err = 0.0;
            end

        case STATE_STOP_PICKUP
            target_xy = pickup_xy;
            v_cmd_mps = 0.0;
            led_rgb = led_blue;
            if stopStartTime < 0
                stopStartTime = t;
            end
            stopTimerDone = (t - stopStartTime) >= stop_hold_sec;
            if stopTimerDone && isStopped
                mode = STATE_GO_DROPOFF;
                stopStartTime = -1.0;
                wpIdx = 1;
                steer_integral = 0.0;
                steer_prev_err = 0.0;
            end

        case STATE_GO_DROPOFF
            target_xy = dropoff_xy;
            v_cmd_mps = cruise_speed_mps;
            led_rgb = led_green;
            stopStartTime = -1.0;
            active_waypoints = wp_dropoff;
            use_waypoints = true;
            if arrived_dropoff
                mode = STATE_STOP_DROPOFF;
                stopStartTime = -1.0;
                wpIdx = 1;
                steer_integral = 0.0;
                steer_prev_err = 0.0;
            end

        case STATE_STOP_DROPOFF
            target_xy = dropoff_xy;
            v_cmd_mps = 0.0;
            led_rgb = led_orange;
            if stopStartTime < 0
                stopStartTime = t;
            end
            stopTimerDone = (t - stopStartTime) >= stop_hold_sec;
            if stopTimerDone && isStopped
                mode = STATE_RETURN_HUB;
                stopStartTime = -1.0;
                wpIdx = 1;
                steer_integral = 0.0;
                steer_prev_err = 0.0;
            end

        case STATE_RETURN_HUB
            target_xy = hub_xy;
            v_cmd_mps = cruise_speed_mps;
            led_rgb = led_green;
            stopStartTime = -1.0;
            active_waypoints = wp_hub;
            use_waypoints = true;
            if arrived_hub
                mode = STATE_WAIT_AT_HUB;
                stopStartTime = -1.0;
                wpIdx = 1;
                steer_integral = 0.0;
                steer_prev_err = 0.0;
            end

        otherwise  % STATE_WAIT_AT_HUB
            target_xy = hub_xy;
            v_cmd_mps = 0.0;
            led_rgb = led_red;
            stopStartTime = -1.0;
    end

    %% ===================== Steering Computation =====================
    if v_cmd_mps > 0 && use_waypoints && ~isempty(active_waypoints)
        % --- Pure pursuit on waypoint path ---
        [steer_pp, wpIdx] = compute_pure_pursuit( ...
            pos_xy, heading_rad, active_waypoints, wpIdx, LOOKAHEAD, WHEELBASE);

        % --- Lane offset correction ---
        % lane_offset > 0 means car is right of center -> steer left (negative)
        % lane_offset < 0 means car is left of center -> steer right (positive)
        lane_correction = -K_LANE * lane_offset;

        % --- Combine: pure pursuit + lane correction ---
        steer_raw = steer_pp + lane_correction;

        % --- PID smoothing on combined steering ---
        steer_err = steer_raw;  % treat desired steering as the "error" signal
        steer_integral = steer_integral + steer_err * dt;
        % Anti-windup: clamp integral
        max_integral = MAX_STEER / max(KI_STEER, 0.01);
        steer_integral = max(-max_integral, min(max_integral, steer_integral));

        steer_deriv = (steer_err - steer_prev_err) / max(dt, 0.001);
        steer_prev_err = steer_err;

        delta_cmd_rad = KP_STEER * steer_err + KI_STEER * steer_integral + KD_STEER * steer_deriv;

        % --- Speed adaptation for turns ---
        abs_steer = abs(delta_cmd_rad);
        if abs_steer > STEER_THRESH_SLOW
            speed_factor = 1.0 - (1.0 - SPEED_TURN_FACTOR) * ...
                min(1.0, (abs_steer - STEER_THRESH_SLOW) / (MAX_STEER - STEER_THRESH_SLOW));
            v_cmd_mps = v_cmd_mps * speed_factor;
        end

    elseif v_cmd_mps > 0
        % --- Fallback: heading-to-goal (for states without waypoints) ---
        dx = target_xy(1) - pos_xy(1);
        dy = target_xy(2) - pos_xy(2);
        angle_to_target = atan2(dy, dx);
        err = atan2(sin(angle_to_target - heading_rad), cos(angle_to_target - heading_rad));
        delta_cmd_rad = 1.5 * err;
    else
        delta_cmd_rad = 0.0;
    end

    %% ===================== Clamp Outputs =====================
    delta_cmd_rad = max(-MAX_STEER, min(MAX_STEER, delta_cmd_rad));
    if ~isfinite(v_cmd_mps) || v_cmd_mps < 0
        v_cmd_mps = 0.0;
    end
    if ~isfinite(delta_cmd_rad)
        delta_cmd_rad = 0.0;
    end

    %% ===================== Write Outputs =====================
    block.OutputPort(1).Data = v_cmd_mps;
    block.OutputPort(2).Data = delta_cmd_rad;
    block.OutputPort(3).Data = led_rgb;
    block.OutputPort(4).Data = mode;
end

%% =====================================================================
%  PURE PURSUIT ON WAYPOINT PATH
%  =====================================================================
function [steerAngle, wpIdxOut] = compute_pure_pursuit(pos, heading, waypoints, wpIdx, lookahead, wheelbase)
    n = size(waypoints, 1);
    wpIdxOut = wpIdx;

    if n < 2
        steerAngle = 0;
        return;
    end

    % Clamp wpIdx to valid range
    if wpIdxOut < 1, wpIdxOut = 1; end
    if wpIdxOut > n, wpIdxOut = n; end

    % Advance waypoint index: skip waypoints that are behind the car
    while wpIdxOut < n
        wx = waypoints(wpIdxOut, 1);
        wy = waypoints(wpIdxOut, 2);
        dx = wx - pos(1);
        dy = wy - pos(2);
        distToWp = sqrt(dx*dx + dy*dy);

        % Also check if waypoint is behind us
        angle_to_wp = atan2(dy, dx);
        heading_diff = abs(atan2(sin(angle_to_wp - heading), cos(angle_to_wp - heading)));

        if distToWp < lookahead * 0.5 || (distToWp < lookahead && heading_diff > pi/2)
            wpIdxOut = wpIdxOut + 1;
        else
            break;
        end
    end

    % Find lookahead point on the path
    lookaheadPt = find_lookahead_point_on_path(pos, waypoints, wpIdxOut, lookahead);

    % Pure pursuit geometry
    dx = lookaheadPt(1) - pos(1);
    dy = lookaheadPt(2) - pos(2);

    % Transform to vehicle local frame
    local_x =  dx * cos(-heading) - dy * sin(-heading);
    local_y =  dx * sin(-heading) + dy * cos(-heading);

    L = sqrt(local_x*local_x + local_y*local_y);
    if L < 0.01
        steerAngle = 0;
        return;
    end

    % Curvature = 2 * y / L^2
    curvature = 2.0 * local_y / (L * L);

    % Steering angle from bicycle model
    steerAngle = atan(wheelbase * curvature);

    % Clamp
    steerAngle = max(-0.5, min(0.5, steerAngle));
end

function pt = find_lookahead_point_on_path(pos, waypoints, startIdx, lookahead)
    n = size(waypoints, 1);

    % Search from startIdx forward for a point at lookahead distance
    for i = max(1, startIdx):n-1
        p1 = waypoints(i, :);
        p2 = waypoints(i+1, :);

        % Line-circle intersection
        d = p2 - p1;
        f = p1 - pos(:)';

        a = dot(d, d);
        b = 2 * dot(f, d);
        c = dot(f, f) - lookahead*lookahead;

        disc = b*b - 4*a*c;
        if disc >= 0
            sqrtDisc = sqrt(disc);
            t1 = (-b - sqrtDisc) / (2*a);
            t2 = (-b + sqrtDisc) / (2*a);

            % Take the furthest valid intersection on the segment
            if t2 >= 0 && t2 <= 1
                pt = p1 + t2 * d;
                return;
            elseif t1 >= 0 && t1 <= 1
                pt = p1 + t1 * d;
                return;
            end
        end
    end

    % Fallback: aim at furthest remaining waypoint within reach, or last waypoint
    pt = waypoints(min(startIdx + 3, n), :);
end

%% =====================================================================
%  WAYPOINT DATA (pre-scaled for x10 full-scale coordinate system)
%  These match the paths in matlab/models/waypoints.txt
%  =====================================================================
function wp = get_waypoints_pickup()
    wp = [
        -12.05, -8.30;
        -10.50, -9.00;
         -8.00, -10.00;
         -6.00, -10.50;
         -4.00, -10.80;
         -2.00, -10.90;
          0.00, -10.95;
          2.00, -10.95;
          4.00, -10.90;
          6.00, -10.85;
          8.00, -10.85;
         10.00, -10.90;
         12.00, -10.92;
         14.00, -10.92;
         16.00, -10.50;
         18.00,  -9.00;
         19.50,  -7.50;
         20.50,  -5.50;
         21.00,  -4.00;
         21.50,  -3.00;
         21.80,  -1.50;
         22.00,   0.00;
         22.10,   2.00;
         22.15,   4.00;
         22.15,   6.00;
         22.15,   8.00;
         22.15,  10.00;
         22.15,  12.00;
         22.15,  14.00;
         22.15,  16.00;
         22.15,  18.00;
         22.10,  20.00;
         22.10,  22.00;
         22.10,  24.00;
         22.10,  26.00;
         22.10,  28.00;
         22.05,  30.00;
         22.00,  32.00;
         21.90,  34.00;
         21.70,  36.00;
         21.40,  38.00;
         21.00,  40.00;
         20.50,  41.50;
         19.80,  43.00;
         18.80,  44.00;
         17.50,  44.50;
         16.00,  44.80;
         14.00,  44.90;
         12.00,  44.92;
         10.00,  44.92;
          8.00,  44.92;
          6.00,  44.92;
          4.00,  44.92;
          2.00,  44.92;
          1.25,  44.50;
          1.25,  43.95
    ];
end

function wp = get_waypoints_dropoff()
    wp = [
         1.25,  43.95;
         0.00,  44.92;
        -2.00,  44.92;
        -4.00,  44.92;
        -6.00,  44.92;
        -8.00,  44.92;
       -10.00,  44.92;
       -12.00,  44.92;
       -14.00,  44.92;
       -16.00,  44.50;
       -17.50,  43.00;
       -18.50,  41.00;
       -19.20,  39.00;
       -19.70,  37.00;
       -19.95,  35.00;
       -20.05,  33.00;
       -20.08,  31.00;
       -20.08,  29.00;
       -20.08,  27.00;
       -20.08,  25.00;
       -20.08,  23.00;
       -20.08,  21.00;
       -20.08,  19.00;
       -19.50,  17.00;
       -19.00,  15.00;
       -18.40,  13.00;
       -17.60,  11.50;
       -16.50,  10.00;
       -15.00,   9.00;
       -13.00,   8.40;
       -11.00,   8.20;
        -9.05,   8.00
    ];
end

function wp = get_waypoints_hub()
    wp = [
        -9.05,   8.00;
        -7.00,   7.50;
        -5.00,   7.20;
        -3.00,   7.10;
        -1.00,   6.00;
         0.00,   4.50;
         0.00,   2.50;
         0.00,   0.50;
         0.00,  -1.50;
         0.00,  -3.50;
         0.50,  -5.50;
         1.00,  -7.00;
         1.50,  -8.50;
         1.20, -10.00;
         0.00, -10.50;
        -2.00, -10.70;
        -4.00, -10.50;
        -6.00, -10.00;
        -8.00,  -9.50;
       -10.00,  -8.80;
       -11.00,  -8.50;
       -12.05,  -8.30
    ];
end

end
