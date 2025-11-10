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
        
        function K = computeGain(obj, A, B)
            % Compute one LQR gain
            K = lqr(A, B, obj.Q, obj.R);
        end

        function delU = computeControls(K, delX)
            delU = -K*delX;
        end

        function [fB, mB] = getForcesMoments(state, rocket, CG, breakPoints, time, log)
                [K, x0, u0] = breakPoints.selectGain(state, time);
                delX = state - x0;
                delU = computeControls(obj, K, delX);
                u = delU + u0;
                if ~isempty(log)
                    log.add(time, 'controlinputs', u)
                end
               [fB, mB] = obj.actuator.getForcesMoments(obj, u, rocket, CG);
        end
    end
end
