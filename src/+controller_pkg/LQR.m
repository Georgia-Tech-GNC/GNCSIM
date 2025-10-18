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
  
        function del_u = compute_controls(K, del_x)
            del_u = -K*del_x;
        end
        
        function [F_b, M_b] = get_forces_and_moments(state, rocket, CG, breakPoints, time)
                [K, x0, u0] = breakPoints.selectGain(state, time);
                del_x = state - x0;
                del_u = compute_controls(obj, K, del_x);
                u = del_u + u0;
               [F_b, M_b] = obj.actuator.get_forces_and_moments(obj, u, rocket, CG);
        end
    end
end
