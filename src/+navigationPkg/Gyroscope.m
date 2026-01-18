classdef Gyroscope < navigationPkg.Sensor&handle

    properties 
        name = "Gyroscope"
        outputDimension = 3;
        noiseCovariance
        outputRate     
        outputProbability       
        observationMatrix
    end

    properties (Constant)
        SENSOR_TYPE = navigationPkg.SensorType.GYROSCOPE
    end

    properties
        w
        ARW
        bias
    end


    methods

        function obj = Gyroscope(gyroParams)

            obj@navigationPkg.Sensor()
            obj.noiseCovariance = gyroParams.noiseCovariance;
            obj.outputRate = gyroParams.outputRate;
            obj.outputProbability = gyroParams.outputProbability;
            obj.observationMatrix = gyroParams.observationMatrix;

            obj.outputPeriod = 1 / obj.outputRate;
            obj.lastOutputTime = 0;
            obj.nextOutputTime = obj.outputPeriod;

            obj.ARW = gyroParams.ARW;
            obj.bias = gyroParams.bias;

        end

        function measurement = getMeasurement(obj, state, stateDot, currentTime)
            
           % if obj.isOutputReady(currentTime)
                obj.w = state(12:14);
                measurement = obj.w; %obj.addNoise(); % Add noise + make []
            %else
             %   measurement = [];
           % end

            obj.scheduleNextOutput(currentTime);

        end

        function noisyGyro = addNoise(obj)
            

            % Fix white noise
            noise = obj.ARW .* sqrt(obj.outputRate) .* randn(3,1);
            noisyGyro = obj.w + obj.bias .* obj.outputRate + noise;

        end

    end
end
