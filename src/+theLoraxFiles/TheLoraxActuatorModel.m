% Main Authors: Karsten Caillet (kcaillet3@gatech.edu)

classdef TheLoraxActuatorModel < controllerPkg.ActuatorModel
    properties
        allocationMatrix % type hinting in abstract class
        rocketFins rocketPkg.RocketFins
        timeConstant (1,1) double {mustBeNonnegative}
        limits struct
    end
    methods 
        % Actuator State is fin deflections in degrees
        function obj = TheOncelerActuatorModel(rocketFins, timeConstant, allocationMatrix, limits)            
            obj.rocketFins = rocketFins; % movable fins object
            obj.timeConstant = timeConstant; % time constant for 1st order model
            obj.allocationMatrix = allocationMatrix;
            obj.limits = limits;
            obj.computeBuildup();
        end

        function [fB, mB] = getForcesMoments(obj, state, actuatorState, currentCg, envConditions)
            arguments
                obj
                state (:,1) double
                actuatorState (:,1) double
                currentCg (1,1) double
                envConditions (1,1) flightPkg.AtmosphericConditions
            end
            [fB, mB] = obj.rocketFins.getForcesMoments(state, actuatorState, currentCg, envConditions);
        end

        function actuatorStateDot = actuatorDynamics(obj, controlInput, actuatorState)
            arguments
                obj
                controlInput (:,1) double
                actuatorState (:,1) double
            end
            % first order actuation model
            % Saturate cmd
            actuatorStateCmd = obj.allocationMatrix * controlInput;
            deltaMax = obj.limits.maxDeflection;
            actuatorStateCmd = min(max(actuatorStateCmd, -deltaMax), deltaMax);
            
            % saturate actuation rate
            actuatorStateDot = 1/obj.timeConstant .* (actuatorStateCmd - actuatorState);
            rateMax = obj.limits.maxRate;
            actuatorStateDot = min(max(actuatorStateDot, -rateMax), rateMax);
            
            % prevent going past limits
            atUpper = actuatorState >=  deltaMax & actuatorStateDot > 0;
            atLower = actuatorState <= -deltaMax & actuatorStateDot < 0;
        
            actuatorStateDot(atUpper | atLower) = 0;
        end

        function computeBuildup(obj)
            % Main Author: David Reynolds
            obj.rocketFins.computeBuildup()
			
			% Added scale factors in order to line up with open rocket
			[~, numFins] = size(obj.rocketFins.fins);
			for ii=1:numFins
				obj.rocketFins.fins(ii).scaleFactor = 1.97;
			end
        end
    end
end
