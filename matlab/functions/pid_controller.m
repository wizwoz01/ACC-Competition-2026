function [output, integral, prev_error] = pid_controller(error, integral, prev_error, Kp, Ki, Kd, dt, output_limits)
%PID_CONTROLLER Compute PID control output
%
%   [output, integral, prev_error] = pid_controller(error, integral, prev_error, Kp, Ki, Kd, dt, output_limits)
%
%   Inputs:
%       error         - Current error (setpoint - measurement)
%       integral      - Accumulated integral term
%       prev_error    - Previous error for derivative calculation
%       Kp            - Proportional gain
%       Ki            - Integral gain
%       Kd            - Derivative gain
%       dt            - Time step (seconds)
%       output_limits - [min, max] output limits (optional)
%
%   Outputs:
%       output     - PID control output
%       integral   - Updated integral term
%       prev_error - Current error (for next iteration)
%
%   Example:
%       Kp = 1.0; Ki = 0.1; Kd = 0.05; dt = 0.05;
%       [steering, integral, prev_error] = pid_controller(lane_error, integral, prev_error, Kp, Ki, Kd, dt, [-0.5, 0.5]);

    % Proportional term
    P = Kp * error;
    
    % Integral term (with anti-windup)
    integral = integral + error * dt;
    I = Ki * integral;
    
    % Derivative term
    derivative = (error - prev_error) / dt;
    D = Kd * derivative;
    
    % Calculate output
    output = P + I + D;
    
    % Apply output limits if provided
    if nargin >= 8 && ~isempty(output_limits)
        output = max(output_limits(1), min(output_limits(2), output));
        
        % Anti-windup: prevent integral from growing when output is saturated
        if output == output_limits(1) || output == output_limits(2)
            integral = integral - error * dt;
        end
    end
    
    % Update previous error
    prev_error = error;
end

