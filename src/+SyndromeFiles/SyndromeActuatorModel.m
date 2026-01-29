classdef ReactionWheelActuatorModel < controllerPkg.ActuatorModel
    properties
        % Wheel params
        Iw
        tauMax
        wMax
        b = 0

        % Bookkeeping for applied moment
        tauApplied = 0

        % Required by framework (some code expects it exists)
        allocationMatrix = eye(3)
    end

    methods
        function obj = ReactionWheelActuatorModel(Iw, tauMax, wMax, b)
            obj.Iw = Iw;
            obj.tauMax = tauMax;
            obj.wMax = wMax;
            if nargin >= 4, obj.b = b; end
        end

        function wdot = actuatorDynamics(obj, controlInput, actuatorState)
            w = actuatorState(1);
            tauCmd = controlInput(1);

            % Saturate torque command
            tau = min(max(tauCmd, -obj.tauMax), obj.tauMax);

            % Enforce wheel speed limit: prevent further acceleration past limit
            % Wheel torque is opposite of body torque
            tauWheel = -tau;
            wdot = (tauWheel - obj.b*w) / obj.Iw;

            if (w >= obj.wMax && wdot > 0), wdot = 0; end
            if (w <= -obj.wMax && wdot < 0), wdot = 0; end

            % Store what we actually applied to the body this step
            obj.tauApplied = -tauWheel; % equals tau
        end

        function [fB, mB] = getForcesMoments(obj, ~, ~, ~, ~)
            fB = zeros(3,1);
            mB = [obj.tauApplied; 0; 0];
        end
    end
end