% Main Authors: Aditya Swami (aswami8@gatech.edu)
%               David Reynolds (dreynolds45@gatech.edu)


classdef Controller < handle
    properties
        breakPoints
    end

    methods
        function obj = Controller(breakPoints)
            obj.breakPoints = breakPoints;
        end

        function gain = getGain(state)
            % Finds gains of the controller based on the state
            
        end

        function u = getControl(state)
            % Finds optimal fin deflection based on state
            
        end
    end
end

% classdef (Abstract) Controller < handle
%     properties 
%         actuator
%     end
% 
%     methods (Abstract)
%         % comments
%         %[fB, mB] = getForcesMoments(state, disturbances, motorParams, CG, breakPoints, time);
%         %gain = computeGain()
%         control = computeControl(state)
% 
%     end
% end
