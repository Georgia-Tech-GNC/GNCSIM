classdef Gyroscope < Sensor

    properties 
        name = "Gyroscope"
        outputDimension = 3
        noiseCovariance
        outputRate = 0;           % Hz
        outputProbability = 1        % 
        observationMatrix
    end

    properties (Constant)
        SENSOR_TYPE = SensorType.GYROSCOPE
    end

    properties
        w
        ARW
        bias
    end

    % Still need to implement sensor error/bias

    methods

        function obj = Gyroscope(ARW, bias)
        
            obj@Sensor()
            obj.ARW = ARW;
            obj.bias = bias;

        end

        function measurement = getMeasurement(obj, state, stateDot, currentTime)

            obj.w = state(11:13);

            if obj.isOutputReady(currentTime)
                measurement = obj.addNoise(); % Add noise
            else
                measurement = [];
            end
        end

        function noisyGyro = addNoise(obj)
            
            noise = obj.ARW .* sqrt(obj.outputRate) .* randn(3,1);
            noisyGyro = obj.w + obj.bias + noise;

        end

    end
end
