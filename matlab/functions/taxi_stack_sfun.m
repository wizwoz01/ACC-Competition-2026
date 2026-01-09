function taxi_stack_sfun(block)
% Level-2 MATLAB S-Function implementing Taxi mission FSM + steering.
%
% Inputs:
%   1) pos_xy      [2x1] double
%   2) heading_rad [1x1] double
%   3) speed_mps   [1x1] double
%   4) t           [1x1] double
%   5) reset       [1x1] double/bool
%
% Outputs:
%   1) v_cmd_mps     [1x1] double
%   2) delta_cmd_rad [1x1] double
%   3) led_rgb       [3x1] uint8
%   4) state         [1x1] uint8
%
% This avoids Stateflow/MATLAB Function blocks, making it robust across
% MATLAB installs where EMChart script injection is unreliable.

setup(block);

function setup(block)
    block.NumDialogPrms = 0;

    block.NumInputPorts  = 5;
    block.NumOutputPorts = 4;

    % Inputs
    block.InputPort(1).Dimensions = 2;
    block.InputPort(1).DatatypeID = 0; % double
    block.InputPort(1).Complexity = 'Real';
    block.InputPort(1).DirectFeedthrough = true;

    block.InputPort(2).Dimensions = 1;
    block.InputPort(2).DatatypeID = 0; % double
    block.InputPort(2).Complexity = 'Real';
    block.InputPort(2).DirectFeedthrough = true;

    block.InputPort(3).Dimensions = 1;
    block.InputPort(3).DatatypeID = 0; % double
    block.InputPort(3).Complexity = 'Real';
    block.InputPort(3).DirectFeedthrough = true;

    block.InputPort(4).Dimensions = 1;
    block.InputPort(4).DatatypeID = 0; % double
    block.InputPort(4).Complexity = 'Real';
    block.InputPort(4).DirectFeedthrough = true;

    block.InputPort(5).Dimensions = 1;
    block.InputPort(5).DatatypeID = 0; % double
    block.InputPort(5).Complexity = 'Real';
    block.InputPort(5).DirectFeedthrough = true;

    % Outputs
    block.OutputPort(1).Dimensions = 1;
    block.OutputPort(1).DatatypeID = 0; % double
    block.OutputPort(1).Complexity = 'Real';

    block.OutputPort(2).Dimensions = 1;
    block.OutputPort(2).DatatypeID = 0; % double
    block.OutputPort(2).Complexity = 'Real';

    block.OutputPort(3).Dimensions = 3;
    block.OutputPort(3).DatatypeID = 3; % uint8
    block.OutputPort(3).Complexity = 'Real';

    block.OutputPort(4).Dimensions = 1;
    block.OutputPort(4).DatatypeID = 3; % uint8
    block.OutputPort(4).Complexity = 'Real';

    % Use inherited sample time.
    block.SampleTimes = [-1 0];

    % Register only Outputs (state is kept via persistent vars inside Outputs).
    block.RegBlockMethod('Outputs', @Outputs);
end

function Outputs(block)
    % Persistent state (avoids DWork which can fail on some MATLAB versions).
    persistent mode stopStartTime initStartTime
    persistent taxiCfg

    % Default taxi config (used if workspace config is missing)
    defaultTaxi = struct();
    defaultTaxi.coord_scale = 1.0;
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

    % Load taxi config from base workspace once (best-effort).
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

    % Pull parameters with safe defaults.
    coord_scale = 1.0;
    try, coord_scale = double(taxiCfg.coord_scale); catch, end
    if ~isfinite(coord_scale) || coord_scale <= 0
        coord_scale = 1.0;
    end

    hub_xy     = taxiCfg.hub_xy(:)     * coord_scale;
    pickup_xy  = taxiCfg.pickup_xy(:)  * coord_scale;
    dropoff_xy = taxiCfg.dropoff_xy(:) * coord_scale;

    arrival_radius = double(taxiCfg.arrival_radius_m) * coord_scale;
    stop_hold_sec = double(taxiCfg.stop_hold_sec);
    init_hold_sec = double(taxiCfg.init_hold_sec);
    cruise_speed_mps = double(taxiCfg.cruise_speed_mps);
    stop_speed_thresh_mps = double(taxiCfg.stop_speed_thresh_mps);

    led_red    = uint8(taxiCfg.led_red(:));
    led_green  = uint8(taxiCfg.led_green(:));
    led_blue   = uint8(taxiCfg.led_blue(:));
    led_orange = uint8(taxiCfg.led_orange(:));

    % State ids
    STATE_INIT         = uint8(0);
    STATE_GO_PICKUP    = uint8(1);
    STATE_STOP_PICKUP  = uint8(2);
    STATE_GO_DROPOFF   = uint8(3);
    STATE_STOP_DROPOFF = uint8(4);
    STATE_RETURN_HUB   = uint8(5);
    STATE_WAIT_AT_HUB  = uint8(6);

    % Inputs
    pos_xy = block.InputPort(1).Data;
    heading_rad = block.InputPort(2).Data;
    speed_mps = block.InputPort(3).Data;
    t = block.InputPort(4).Data;
    reset = block.InputPort(5).Data;

    % Initialize persistent state on first call or reset.
    if isempty(mode) || (~isempty(reset) && reset(1) ~= 0)
        mode = STATE_INIT;
        stopStartTime = -1.0;
        if isfinite(t)
            initStartTime = t;
        else
            initStartTime = 0.0;
        end
    end

    % Default outputs
    target_xy = hub_xy;
    v_cmd_mps = 0.0;
    led_rgb = led_red;

    % Validate inputs
    pos_xy = pos_xy(:);
    if numel(pos_xy) ~= 2 || any(~isfinite(pos_xy))
        mode = STATE_WAIT_AT_HUB;
        target_xy = hub_xy;
        v_cmd_mps = 0.0;
        led_rgb = led_red;
        stopStartTime = -1.0;
    else
        % Arrival checks
        dxp = pos_xy(1) - pickup_xy(1); dyp = pos_xy(2) - pickup_xy(2);
        dxd = pos_xy(1) - dropoff_xy(1); dyd = pos_xy(2) - dropoff_xy(2);
        dxh = pos_xy(1) - hub_xy(1);    dyh = pos_xy(2) - hub_xy(2);

        arrived_pickup  = sqrt(dxp*dxp + dyp*dyp) <= arrival_radius;
        arrived_dropoff = sqrt(dxd*dxd + dyd*dyd) <= arrival_radius;
        arrived_hub     = sqrt(dxh*dxh + dyh*dyh) <= arrival_radius;

        isStopped = true;
        if isfinite(speed_mps)
            isStopped = speed_mps <= stop_speed_thresh_mps;
        end

        % Stop timer (only used in stop states)
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
                end

            case STATE_GO_PICKUP
                target_xy = pickup_xy;
                v_cmd_mps = cruise_speed_mps;
                led_rgb = led_green;
                stopStartTime = -1.0;
                if arrived_pickup
                    mode = STATE_STOP_PICKUP;
                    stopStartTime = -1.0;
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
                end

            case STATE_GO_DROPOFF
                target_xy = dropoff_xy;
                v_cmd_mps = cruise_speed_mps;
                led_rgb = led_green;
                stopStartTime = -1.0;
                if arrived_dropoff
                    mode = STATE_STOP_DROPOFF;
                    stopStartTime = -1.0;
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
                end

            case STATE_RETURN_HUB
                target_xy = hub_xy;
                v_cmd_mps = cruise_speed_mps;
                led_rgb = led_green;
                stopStartTime = -1.0;
                if arrived_hub
                    mode = STATE_WAIT_AT_HUB;
                    stopStartTime = -1.0;
                end

            otherwise % STATE_WAIT_AT_HUB
                target_xy = hub_xy;
                v_cmd_mps = 0.0;
                led_rgb = led_red;
                stopStartTime = -1.0;
        end
    end

    % Steering: heading-to-goal
    dx = target_xy(1) - pos_xy(1);
    dy = target_xy(2) - pos_xy(2);
    angle_to_target = atan2(dy, dx);
    err = atan2(sin(angle_to_target - heading_rad), cos(angle_to_target - heading_rad));
    K = 1.5;
    delta_cmd_rad = K * err;

    % Output (saturations are handled by Simulink Sat blocks, but keep safe)
    if ~isfinite(v_cmd_mps) || v_cmd_mps < 0
        v_cmd_mps = 0.0;
    end

    block.OutputPort(1).Data = v_cmd_mps;
    block.OutputPort(2).Data = delta_cmd_rad;
    block.OutputPort(3).Data = led_rgb;
    block.OutputPort(4).Data = mode;
end

end
