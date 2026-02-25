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
        idx = StateIndices
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

                obj.w = state(obj.idx.omega);
                measurement = obj.addNoise();
                

        end

        function noisyGyro = addNoise(obj)

            noisyGyro = obj.w + obj.bias + (obj.ARW/sqrt(obj.outputPeriod)) .* randn(3,1);

        end

    end
end
