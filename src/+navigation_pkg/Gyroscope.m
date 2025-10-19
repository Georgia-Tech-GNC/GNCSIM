classdef Gyroscope < Sensor

    properties 
        name = "Gyroscope"
        outputDimension = 3;
        noiseCovariance
        outputRate     % Hz
        outputProbability        % 
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


    methods

        function obj = Gyroscope(gyro_params)
        
            obj@Sensor()
            obj.noiseCovariance = gyro_params.noiseCovariance;
            obj.outputRate = gyro_params.outputRate;
            obj.outputProbability = gyro_params.outputProbability;
            obj.observationMatrix = gyro_params.observationMatrix;

            obj.ARW = gyro_params.ARW;
            obj.bias = gyro_params.bias;

        end

        function measurement = getMeasurement(obj, state, stateDot)%, currentTime)

           % if obj.isOutputReady(currentTime)
                obj.w = state(11:13);
                measurement = obj.w % obj.addNoise(); % Add noise
           % else
                %measurement = [];
           % end
        end

        function noisyGyro = addNoise(obj)
            
            noise = obj.ARW .* sqrt(obj.outputRate) .* randn(3,1);
            noisyGyro = obj.w + obj.bias + noise;

        end

    end
end
