classdef ReactionWheelActuatorModel < controllerPkg.ActuatorModel
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here

    properties
        Property1
    end

    methods
        function obj = ReactionWheelActuatorModel(inputArg1,inputArg2)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            obj.Property1 = inputArg1 + inputArg2;
        end

        function [mb, fb] = getForcesMoments(obj, controlInput, rocket, CG)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            mb = zeros(3,1);
            fb = [-controlInput; 0; 0];
        end

        function [actuatorStateDot] = actuatorDynamics(obj, actuatorState) 
            
            

        end
    end
end