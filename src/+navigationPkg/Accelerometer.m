classdef Accelerometer < navigationPkg.Sensor&handle
    
    properties 
        name = "Accelerometer"
        outputDimension = 3
        noiseCovariance
        outputRate           
        outputProbability      
        observationMatrix
    end

    properties (Constant)
        SENSOR_TYPE = navigationPkg.SensorType.ACCELEROMETER
        idx = StateIndices
    end
    
    properties
        
        accelB
        angVelB
        q
        leverArm
        bias
        SF
        VRW

    end

    methods

        function obj = Accelerometer(accParams)

            obj@navigationPkg.Sensor()
            obj.noiseCovariance = accParams.noiseCovariance;
            obj.outputRate = accParams.outputRate;
            obj.outputProbability = accParams.outputProbability;
            obj.observationMatrix = accParams.observationMatrix;

            obj.outputPeriod = 1 / obj.outputRate;
            obj.lastOutputTime = 0;
            obj.nextOutputTime = obj.outputPeriod;

            obj.leverArm = accParams.leverArm;
            obj.SF = accParams.SF;
            obj.bias = accParams.bias;
            obj.VRW = accParams.VRW;

        end

        function measurement = getMeasurement(obj, state, stateDot, currentTime)

                obj.q = state(obj.idx.quat);
                obj.angVelB = state(obj.idx.omega);

                obj.accelB = stateDot(obj.idx.vel); % 
    
                accelIMU = obj.accLeverArm();
                measurement = obj.addNoise(accelIMU);

        end

        function accelIMU = accLeverArm(obj)
            % Lever-arm corrected acceleration
            w = obj.angVelB;
            r = obj.leverArm;
            r = zeros(3,1); % placeholder
            velCG = zeros(3,1); % Placeholder for future coupling term
            correction = cross(w, velCG) + cross(w, cross(w, r));
            cBI = Utils.dirCosine(obj.q);
            accelIMU = obj.accelB - cBI * correction + cBI * [0; 0; 9.81];
        end

        function noisyAccel = addNoise(obj, accelIMU)
            % Add noise, bias, and scale factor
            noisyAccel = diag(1+obj.SF) * accelIMU + obj.bias + (obj.VRW/sqrt(obj.outputPeriod)) .* randn(3,1);

        end

    end
end
