classdef Accelerometer < handle
    
    properties 
        name = "Accelerometer"
        outputDimension = 3
        noiseCovariance
        outputRate = 0;           % Hz
        outputProbability = 1        % 
        observationMatrix
    end

    properties (Constant)
        SENSOR_TYPE = SensorType.ACCELEROMETER
    end
    
    properties
        
        accelB
        angVelB
        qBI
        leverArm
        f
        SF
        VRW

    end

    methods

        function obj = Accelerometer(state, stateDot, leverArm, f, SF, VRW)
            [obj.accelB] = stateDot(4:6); % Need to check
            [obj.qBI] = state(7:10);
            [obj.angVelB] = state(11:13);
            obj.leverArm = leverArm;
            obj.f = f;
            obj.SF = SF;
            obj.VRW

        end

        function [accelIMU] = accLeverArm(obj)
            % a = w x v (Velocity of IMU relative to CG or -CG)+ w x (w x r CG->IMU)
            velCG = 0; % Need to add
            accelIMU = obj.accelB - Utils.quatRotate(obj.qBI,cross(obj.angVelB, velCG) + cross(obj.angVelB, cross(obj.angVelB, obj.leverArm)));                
        end

        function [noisyAccel] = addNoise(obj, accelIMU)
            noise = obj.VRW .* sqrt(obj.f) .* randn(3);
            noisyAccel = (1 + obj.SF) .* accelIMU + bias + noise;
        end
    end
end
