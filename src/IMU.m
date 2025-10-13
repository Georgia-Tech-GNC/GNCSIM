classdef IMU < handle
    properties

    end

    methods

        function obj = IMU()
            
        end

        function [accelBody] = getAcc(obj, trueAccelNED, trueAngVelBody, qBI)
            % a = w x v (Velocity of IMU relative to CG or -CG)+ w x (w x r CG->IMU)
                

        end

        
    end
end
