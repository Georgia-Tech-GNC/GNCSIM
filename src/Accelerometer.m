classdef Accelerometer < Sensor
    
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
        bias
        SF
        VRW

    end

    methods

        function obj = Accelerometer(leverArm, VRW, SF, bias)

            obj@Sensor()
            obj.leverArm = leverArm;
            obj.SF = SF;
            obj.bias = bias;
            obj.VRW = VRW;

        end

        function accelIMU = accLeverArm(obj)
            % Lever-arm corrected acceleration
            w = obj.angVelB;
            r = obj.leverArm;
            velCG = zeros(3,1); % Placeholder for future coupling term
            correction = cross(w, velCG) + cross(w, cross(w, r));
            accelIMU = obj.accelB - Utils.quatRotate(obj.qBI, correction);
        end

        function noisyAccel = addNoise(obj, accelIMU)
            % Add noise, bias, and scale factor
            noise = obj.VRW .* sqrt(obj.outputRate) .* randn(3,1);
            noisyAccel = (1 + obj.SF) .* accelIMU + obj.bias + noise;
        end

        function measurement = getMeasurement(obj, state, stateDot, currentTime)
            obj.qBI = state(7:10);
            obj.angVelB = state(11:13);
            obj.accelB = stateDot(4:6); 

            accelIMU = obj.accLeverArm();
            measurement = obj.addNoise(accelIMU);
        end
    end
end
