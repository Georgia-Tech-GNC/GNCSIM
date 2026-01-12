% Main Authors: Karsten Caillet (kcaillet3@gatech.edu)

classdef TheOncelerActuatorModel < controllerPkg.ActuatorModel
    properties
        allocationMatrix
        rocketFins
        timeConstant
    end
    methods 
        % Actuator State is fin deflections in degrees
        function obj = TheOncelerActuatorModel(rocketFins, timeConstant, allocationMatrix)            
            obj.rocketFins = rocketFins; % movable fins object
            obj.timeConstant = timeConstant; % time constant for 1st order model
            obj.allocationMatrix = allocationMatrix;
            obj.computeBuildup();
        end

        function [fB, mB] = getForcesMoments(obj, state, actuatorState, currentCg, disturbances)
            [fB, mB] = obj.rocketFins.getForcesMoments(state, actuatorState, currentCg, disturbances);
        end

        function actuatorStateDot = actuatorDynamics(obj, controlInput, actuatorState)
            % first order actuation model
            actuatorStateCmd = obj.allocationMatrix * controlInput;
            actuatorStateDot = 1/obj.timeConstant .* (actuatorStateCmd - actuatorState);
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
