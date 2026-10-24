% Gimbal TVC actuator model
% Actuator state = [gimbal pitch; gimbal yaw] in degrees.
% Sign convention: positive gimbal pitch -> positive body pitch moment (+My),
%                  positive gimbal yaw   -> positive body yaw moment   (+Mz).
% Returns only the CHANGE from axial thrust, because RocketDynamics.computeThrust
% already applies [T; 0; 0].

classdef GimbalActuatorModel < controllerPkg.ActuatorModel
    properties
        allocationMatrix                              % [2 x 3] maps [roll; pitch; yaw] -> [dPitch; dYaw]
        xPivot (1,1) double                           % gimbal pivot (ball centre) from nose tip, +aft [m]
        timeConstant (1,1) double {mustBeNonnegative} % 1st-order actuator time constant [s]
        limits struct                                 % .maxDeflection [deg], .maxRate [deg/s]
    end

    methods
        function obj = GimbalActuatorModel(xPivot, timeConstant, allocationMatrix, limits)
            obj.xPivot = xPivot;
            obj.timeConstant = timeConstant;
            obj.allocationMatrix = allocationMatrix;
            obj.limits = limits;
        end

        function [fB, mB] = getForcesMoments(obj, state, actuatorState, currentCg, envConditions, thrust) %#ok<INUSL>
            arguments
                obj
                state (:,1) double
                actuatorState (:,1) double
                currentCg (1,1) double
                envConditions (1,1) flightPkg.AtmosphericConditions
                thrust (1,1) double = 0
            end

            dp = deg2rad(actuatorState(1));   % gimbal pitch
            dy = deg2rad(actuatorState(2));   % gimbal yaw

            % Unit thrust direction in body frame (norm = 1)
            dirB = [cos(dp)*cos(dy);
                   -cos(dp)*sin(dy);
                    sin(dp)];

            fFull = thrust * dirB;

            % Lever arm from CG to pivot (pivot aft of CG -> negative body x)
            r = [currentCg - obj.xPivot; 0; 0];

            mB = cross(r, fFull);             % axial part has zero moment (r is along x)
            fB = fFull - [thrust; 0; 0];      % delta only; axial thrust is in computeThrust
        end

        function actuatorStateDot = actuatorDynamics(obj, controlInput, actuatorState)
            arguments
                obj
                controlInput (:,1) double
                actuatorState (:,1) double
            end
            % Saturated command
            cmd = obj.allocationMatrix * controlInput;
            deltaMax = obj.limits.maxDeflection;
            cmd = min(max(cmd, -deltaMax), deltaMax);

            % First-order response with rate limit
            actuatorStateDot = (cmd - actuatorState) / obj.timeConstant;
            rateMax = obj.limits.maxRate;
            actuatorStateDot = min(max(actuatorStateDot, -rateMax), rateMax);

            % Hold at position limits
            atUpper = actuatorState >=  deltaMax & actuatorStateDot > 0;
            atLower = actuatorState <= -deltaMax & actuatorStateDot < 0;
            actuatorStateDot(atUpper | atLower) = 0;
        end
    end
end