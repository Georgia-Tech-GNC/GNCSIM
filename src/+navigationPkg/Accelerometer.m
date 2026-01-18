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
            
            %if obj.isOutputReady(currentTime)
                obj.q = state(8:11);
                obj.angVelB = state(12:14);

                obj.accelB = stateDot(5:7); % 
    
                measurement = obj.accLeverArm();
                 %obj.addNoise(accelIMU);
           % else
             %   measurement = [];
            %end

            obj.scheduleNextOutput(currentTime);

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
            % Recheck this why was randn multiplied times 
            noisyAccel = (1 + obj.SF) .* accelIMU .* obj.outputRate + obj.bias .* obj.outputRate + obj.VRW .* sqrt(obj.outputRate) + randn(3,1);

        end

    end
end
