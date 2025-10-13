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
        wBI
    end

    % Still need to implement sensor error/bias

    methods

        function obj = Gyroscope(state)
        
            obj@Sensor()
            obj.wBI = state(11:13);
            
            obj.observationMatrix = 0; % Most likely zeros, noise covaraince doesnt really matter in this case

        end

        function measurement = getMeasurement()
             if obj.isOutputReady(currentTime)
                measurement = ; % Add noise
            else
                measurement = [];
            end
        end

        function noisyMeas = addNoise()

            
        end

    end
end
