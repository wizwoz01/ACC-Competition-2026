function [target_xy, desired_speed_mps, led_rgb, state] = taxi_mission_fsm(pos_xy, speed_mps, t, reset, taxi)
%TAXI_MISSION_FSM Mission/state machine for the ACC taxi scenario.
%
% Implements the sequence:
%   1) Start at taxi hub, LED Red
%   2) LED Green, navigate to pickup [0.125, 4.395]
%   3) Full stop, LED Blue (pickup)
%   4) LED Green, navigate to dropoff [-0.905, 0.800]
%   5) Full stop, LED Orange (dropoff)
%   6) Navigate back to hub, then LED Red (await)
%
% Inputs:
%   pos_xy    - [2x1] current position (meters)
%   speed_mps - current speed estimate (m/s). If NaN/empty, stop checks ignore it.
%   t         - current time (seconds), e.g., Simulink Clock
%   reset     - boolean reset (true to restart mission)
%   taxi      - struct from config/taxi_scenario_params.m
%
% Outputs:
%   target_xy         - [2x1] current navigation target
%   desired_speed_mps - scalar speed command (m/s)
%   led_rgb           - uint8(3x1) LED color
%   state             - uint8 mission state id

    % State IDs (uint8 for Simulink friendliness)
    STATE_INIT            = uint8(0);
    STATE_GO_PICKUP       = uint8(1);
    STATE_STOP_PICKUP     = uint8(2);
    STATE_GO_DROPOFF      = uint8(3);
    STATE_STOP_DROPOFF    = uint8(4);
    STATE_RETURN_HUB      = uint8(5);
    STATE_WAIT_AT_HUB     = uint8(6);

    persistent mode stopStartTime
    if isempty(mode) || (nargin >= 4 && ~isempty(reset) && reset)
        mode = STATE_INIT;
        stopStartTime = -1;
    end

    % Defaults
    target_xy = taxi.hub_xy;
    desired_speed_mps = 0.0;
    led_rgb = taxi.led_red;
    state = mode;

    % Input sanitization
    if nargin < 2 || isempty(speed_mps)
        speed_mps = NaN;
    end
    if nargin < 3 || isempty(t)
        t = 0.0;
    end

    if numel(pos_xy) ~= 2 || any(~isfinite(double(pos_xy)))
        % No valid localization: stop and show red.
        mode = STATE_WAIT_AT_HUB;
        state = mode;
        target_xy = taxi.hub_xy;
        desired_speed_mps = 0.0;
        led_rgb = taxi.led_red;
        return;
    end

    pos_xy = double(pos_xy(:));

    % Helpers
    function d = dist_to(pt)
        dx = pos_xy(1) - double(pt(1));
        dy = pos_xy(2) - double(pt(2));
        d = sqrt(dx*dx + dy*dy);
    end

    function stopped = is_stopped()
        if ~isfinite(speed_mps)
            stopped = true; % if no speed signal, assume ok for timed stop.
        else
            stopped = speed_mps <= taxi.stop_speed_thresh_mps;
        end
    end

    function done = stop_timer_done()
        if stopStartTime < 0
            stopStartTime = t;
        end
        done = (t - stopStartTime) >= taxi.stop_hold_sec;
    end

    arrived_pickup  = dist_to(taxi.pickup_xy)  <= taxi.arrival_radius_m;
    arrived_dropoff = dist_to(taxi.dropoff_xy) <= taxi.arrival_radius_m;
    arrived_hub     = dist_to(taxi.hub_xy)     <= taxi.arrival_radius_m;

    switch mode
        case STATE_INIT
            % Show red briefly at start.
            target_xy = taxi.hub_xy;
            desired_speed_mps = 0.0;
            led_rgb = taxi.led_red;

            if t >= taxi.init_hold_sec
                mode = STATE_GO_PICKUP;
                stopStartTime = -1;
            end

        case STATE_GO_PICKUP
            target_xy = taxi.pickup_xy;
            desired_speed_mps = taxi.cruise_speed_mps;
            led_rgb = taxi.led_green;

            if arrived_pickup
                mode = STATE_STOP_PICKUP;
                stopStartTime = -1;
            end

        case STATE_STOP_PICKUP
            target_xy = taxi.pickup_xy;
            desired_speed_mps = 0.0;
            led_rgb = taxi.led_blue;

            if stop_timer_done() && is_stopped()
                mode = STATE_GO_DROPOFF;
                stopStartTime = -1;
            end

        case STATE_GO_DROPOFF
            target_xy = taxi.dropoff_xy;
            desired_speed_mps = taxi.cruise_speed_mps;
            led_rgb = taxi.led_green;

            if arrived_dropoff
                mode = STATE_STOP_DROPOFF;
                stopStartTime = -1;
            end

        case STATE_STOP_DROPOFF
            target_xy = taxi.dropoff_xy;
            desired_speed_mps = 0.0;
            led_rgb = taxi.led_orange;

            if stop_timer_done() && is_stopped()
                mode = STATE_RETURN_HUB;
                stopStartTime = -1;
            end

        case STATE_RETURN_HUB
            target_xy = taxi.hub_xy;
            desired_speed_mps = taxi.cruise_speed_mps;
            led_rgb = taxi.led_green;

            if arrived_hub
                mode = STATE_WAIT_AT_HUB;
                stopStartTime = -1;
            end

        case STATE_WAIT_AT_HUB
            target_xy = taxi.hub_xy;
            desired_speed_mps = 0.0;
            led_rgb = taxi.led_red;

        otherwise
            mode = STATE_WAIT_AT_HUB;
            target_xy = taxi.hub_xy;
            desired_speed_mps = 0.0;
            led_rgb = taxi.led_red;
    end

    state = mode;
end
