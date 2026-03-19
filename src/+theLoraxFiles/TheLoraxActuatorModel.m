% Main Authors: Karsten Caillet (kcaillet3@gatech.edu)

classdef TheLoraxActuatorModel < controllerPkg.ActuatorModel
    properties
        allocationMatrix    % 2x3: maps [roll, pitch, yaw] -> [delta_y, delta_z] gimbal angles
        motor rocketPkg.Motor
        motorPosition (1,1) double  % nozzle/gimbal point distance from nose [m]
        timeConstant (1,1) double {mustBeNonnegative}
        limits struct
    end
    methods
        function obj = TheLoraxActuatorModel(motor, motorPosition, timeConstant, allocationMatrix, limits)
            obj.harnessesMotor = true;
            obj.motor = motor;
            obj.motorPosition = motorPosition;
            obj.timeConstant = timeConstant;
            obj.allocationMatrix = allocationMatrix;
            obj.limits = limits;
        end

        function [fB, mB] = getForcesMoments(obj, state, actuatorState, currentCg, envConditions, time)
            arguments
                obj
                state (:,1) double
                actuatorState (:,1) double
                currentCg (1,1) double
                envConditions (1,1) flightPkg.AtmosphericConditions
                time (1,1) double {mustBeNonnegative}
            end
            % actuatorState = [delta_y; delta_z] gimbal angles in degrees
            dy = deg2rad(actuatorState(1));
            dz = deg2rad(actuatorState(2));

            % Scalar thrust from motor
            T = obj.motor.getThrust(time) * obj.motor.useThrust;

            % Rotate thrust vector by gimbal angles
            fB = T * [cos(dy)*cos(dz); sin(dz); -sin(dy)*cos(dz)];

            % Moment arm from CG to nozzle gimbal point
            r = [obj.motorPosition - currentCg; 0; 0];
            mB = cross(r, fB);
        end

        function actuatorStateDot = actuatorDynamics(obj, controlInput, actuatorState)
            arguments
                obj
                controlInput (:,1) double
                actuatorState (:,1) double
            end
            % First order actuation model for 2-element gimbal state
            % Saturate command
            actuatorStateCmd = obj.allocationMatrix * controlInput;
            deltaMax = obj.limits.maxDeflection;
            actuatorStateCmd = min(max(actuatorStateCmd, -deltaMax), deltaMax);

            % Saturate actuation rate
            actuatorStateDot = 1/obj.timeConstant .* (actuatorStateCmd - actuatorState);
            rateMax = obj.limits.maxRate;
            actuatorStateDot = min(max(actuatorStateDot, -rateMax), rateMax);

            % Prevent going past limits
            atUpper = actuatorState >=  deltaMax & actuatorStateDot > 0;
            atLower = actuatorState <= -deltaMax & actuatorStateDot < 0;

            actuatorStateDot(atUpper | atLower) = 0;
        end
    end
end
