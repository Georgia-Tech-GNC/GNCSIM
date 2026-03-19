% Main Authors: Karsten Caillet (kcaillet3@gatech.edu)

% "Abstract" ActuatorModel Class"
% (MATLAB doesn't handle type hinting with real abtract classes)

classdef ActuatorModel < handle

    % properties (Abstract)
    %     allocationMatrix (:,:) double % maps controls to deflections
    % end

    properties
        harnessesMotor (1,1) logical = false  % true if actuator redirects motor thrust (e.g. TVC)
    end

    methods
        function [fB, mB] = getForcesMoments(obj, state, actuatorState, currentCg, envConditions, time) %#ok<INUSD>
            error('ActuatorModel:getForcesMoments', ...
                  'getForcesMoments must be implemented in a subclass.');

        end
        
        function actuatorStateDot = actuatorDynamics(obj, controlInput, actuatorState) %#ok<INUSD>
            error('ActuatorModel:actuatorDynamics', ...
                  'actuatorDynamics must be implemented in a subclass.');
        end
    end
end