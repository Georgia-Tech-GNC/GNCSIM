% Main Authors: Karsten Caillet (kcaillet3@gatech.edu)

% "Abstract" ActuatorModel Class"
% (MATLAB doesn't handle type hinting with real abtract classes)

classdef ActuatorModel < handle

    properties
        allocationMatrix (:,:) double % maps controls to deflections
        motor rocketPkg.Motor = rocketPkg.Motor.empty()
    end

    methods
        function [fB, mB] = getForcesMoments(obj, state, actuatorState, currentCg, atmoConditions, time) %#ok<INUSD>
            error('ActuatorModel:getForcesMoments', ...
                  'getForcesMoments must be implemented in a subclass.');

        end
        
        function actuatorStateDot = actuatorDynamics(obj, controlInput, actuatorState) %#ok<INUSD>
            error('ActuatorModel:actuatorDynamics', ...
                  'actuatorDynamics must be implemented in a subclass.');
        end
    end
end