classdef LQR < Controller
    properties
        Q       % State weighting matrix
        R       % Control weighting matrix
        gains   % Struct with K for each trim point
        actuator % Actuator Model Object
    end
    
    methods
        function obj = LQR(Q, R, actuator)
            % Constructor for LQR Controller
            obj.Q = Q;
            obj.R = R;
            obj.actuator = actuator;
        end
        
        function K = compute_gain(obj, A, B)
            % Compute one LQR gain
            K = lqr(A, B, obj.Q, obj.R);
        end
        
        function [K, x0, u0] = selectGain(obj, x, t, trimmedPoints, referenceStates)
            % Author: Karsten Caillet

            % LQR_configuration_selector Selects the appropriate LQR configuration based on the current state and time.
            %
            % INPUTS:
            % obj - The object containing trimmed_points and reference_states structures with pre-computed data.
            % x   - Current state vector (9x1) consisting of:
            %       [u, v, w, p, q, r, q1, q2, q3], where:
            %           u, v, w: Velocity components (m/s)
            %           p, q, r: Angular velocity components (rad/s)
            %           q1, q2, q3: Quaternion components representing orientation.
            % t   - Current time (scalar, in seconds). The function rounds `t` to the nearest integer.
            %
            % OUTPUTS:
            % K   - LQR gain matrix corresponding to the closest matching trimmed condition.
            % x0  - Reference state vector corresponding to the matched condition.
            % u0  - Reference control vector corresponding to the matched condition.
    
            % Variables from input
            u = x(1);
            v = x(2);
            w = x(3);
            q1 = x(7);
            q2 = x(8);
            q3 = x(9);
        
            % Set t to a nonzero integer (1-12)
            t = round(t);
            if (t < 1.0)
                t = 1.0;
            end
        
            % Get airspeed
            airspeed = sqrt(u^2 + v^2 + w^2);
        
            % Find the matching time index in trimmed_points struct
            t_values = round([trimmedPoints.time]);  % Extract all t values into an array
            ind = (t_values == t);  % Compare t_values to t
    
            % Check if any time matches
            if ~any(ind)
                K = zeros(size([trimmedPoints(1).K]));
                x0 = zeros(size(x));
            end
        
            % Calculate the differences for the matching rows
            speedDiff = arrayfun(@(s) abs(sqrt(s.states(1)^2 + s.states(2)^2 + s.states(3)^2) - airspeed), trimmedPoints(ind));
            q1Diff = arrayfun(@(s) abs(s.states(8) - q1), trimmedPoints(ind));
            q2Diff = arrayfun(@(s) abs(s.states(9) - q2), trimmedPoints(ind));
            q3Diff = arrayfun(@(s) abs(s.states(10) - q3), trimmedPoints(ind));
            distance = sqrt(speedDiff.^2 + q1Diff.^2 + q2Diff.^2 + q3Diff.^2);
        
            % Find the minimum distance and its index
            [~, min_index] = min(distance);
        
            % Extract the corresponding K gain
            K_sublist = {trimmedPoints(ind).K};
            K = K_sublist{min_index};

            % Find the matching time index in reference_state struct
            t_values = round([referenceStates.time]);  % Extract all t values into an array
            ind = (t_values == t);  % Compare t_values to t
            
            % Construct x0 (reference state vector)
            reference_states_index = logical([1 1 1 1 1 1 0 1 1 1 0 0 0]); % Getting u, v, w, p, q, r, q1, q2, q3
            x0 = referenceStates(ind).states(reference_states_index);  % Getting reference states for the correct second
            
            % Construct u0 (reference control vector)
            u0 = referenceStates(ind).states(end-2:end); % Getting reference controls for the correct second
        end
  
        function del_u = compute_controls(obj, K, del_x)
            del_u = -K*del_x;
        end
        
        function [F_b, M_b] = get_forces_and_moments(state, disturbances, motor_params, CG, breakPoints, time)
                [K, x0, u0] = selectGain(obj, state, time, breakPoints, referenceStates);
                del_x = state - x0;
                del_u = compute_controls(obj, K, del_x);
                u = del_u + u0;
               [F_b, M_b] = obj.actuator.get_forces_and_moments(obj, u, state, disturbances, CG, motor_params);
        end
    end
end
