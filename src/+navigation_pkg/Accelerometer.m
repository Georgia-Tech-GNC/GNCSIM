classdef Accelerometer < Sensor
    
    properties 
        name = "Accelerometer"
        outputDimension = 3
        noiseCovariance
        outputRate           
        outputProbability      
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

        function obj = Accelerometer(acc_params)

            obj@Sensor()
            obj.noiseCovariance = acc_params.noiseCovariance;
            obj.outputRate = acc_params.outputRate;
            obj.outputProbability = acc_params.outputProbability;
            obj.observationMatrix = acc_params.observationMatrix;

            obj.outputPeriod = 1 / obj.outputRate;
            obj.lastOutputTime = 0;
            obj.nextOutputTime = obj.outputPeriod;

            obj.leverArm = acc_params.leverArm;
            obj.SF = acc_params.SF;
            obj.bias = acc_params.bias;
            obj.VRW = acc_params.VRW;

        end

        function measurement = getMeasurement(obj, state, stateDot, currentTime)
            
            if obj.isOutputReady(currentTime)
                obj.qBI = state(8:11);
                obj.angVelB = state(12:14);
                obj.accelB = stateDot(5:7); 
    
                accelIMU = obj.accLeverArm();
                measurement = obj.addNoise(accelIMU);
            else
                measurement = [];
            end

            obj.scheduleNextOutput(currentTime);

        end

        function accelIMU = accLeverArm(obj)
            % Lever-arm corrected acceleration
            w = obj.angVelB;
            r = obj.leverArm;
            r = zeros(3,1); % placeholder
            velCG = zeros(3,1); % Placeholder for future coupling term
            correction = cross(w, velCG) + cross(w, cross(w, r));
            accelIMU = obj.accelB - Utils.quatRotate(obj.qBI, correction);
        end

        function noisyAccel = addNoise(obj, accelIMU)
            % Add noise, bias, and scale factor
            noise = obj.VRW .* sqrt(obj.outputRate) .* randn(3,1);
            noisyAccel = (1 + obj.SF) .* accelIMU + obj.bias + noise;

        end

    end
end
