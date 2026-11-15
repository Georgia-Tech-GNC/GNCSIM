classdef (Abstract) AeroSource < handle & matlab.mixin.Copyable
    % What RocketDynamics calls for aero forces and moments


    methods (Abstract)
        % fB, mB in the body frame, moments about the current CG
        [fB, mB] = getForcesMoments(obj, state, currentCg, envConditions, powerOn)

        computeBuildup(obj)
    end

    methods
        function relink(obj, body, rocketFins)
            % datcom
        end
    end
end