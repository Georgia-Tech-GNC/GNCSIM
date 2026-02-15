classdef Gyroscope < navigationPkg.Sensor & handle

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
        w      % true angular rate (3x1) [rad/s]
        ARW    % angle random walk (3x1) [rad/sqrt(s)]
        bias   % constant bias (3x1) [rad/s]
    end

    methods
        function obj = Gyroscope(gyroParams)
            obj@navigationPkg.Sensor();

            obj.noiseCovariance   = gyroParams.noiseCovariance;
            obj.outputRate        = gyroParams.outputRate;
            obj.outputProbability = gyroParams.outputProbability;
            obj.observationMatrix = gyroParams.observationMatrix;

            obj.outputPeriod  = 1 / obj.outputRate;
            obj.lastOutputTime = 0;
            obj.nextOutputTime = obj.outputPeriod;

            obj.ARW  = gyroParams.ARW;
            obj.bias = gyroParams.bias;
        end

        function measurement = getMeasurement(obj, state, stateDot, currentTime) %#ok<INUSD>
            % Optional output-rate gating (disabled by default):
            %{
            if ~obj.isOutputReady(currentTime)
                measurement = [];
                return;
            end
            %}

            obj.w = state(obj.idx.omega);      % 3x1 [wx; wy; wz]
            measurement = obj.addNoise();      % apply bias + noise
            obj.scheduleNextOutput(currentTime);
        end

        function noisyGyro = addNoise(obj)
            % Returns noisy gyro measurement in rad/s.
            % Uses noiseCovariance if provided; otherwise uses ARW + outputRate.

            % Bias (rad/s)
            b = obj.bias;

            % Noise (rad/s)
            R = obj.noiseCovariance;
            if ~isempty(R) && all(size(R) == [3 3])
                if all(R(:) == 0)
                    noise = zeros(3,1);
                else
                    [L,p] = chol(R,'lower');
                    if p ~= 0
                        error("Gyroscope:BadNoiseCov", "noiseCovariance must be symmetric PSD.");
                    end
                    noise = L * randn(3,1);
                end
            else
                % ARW is typically specified in rad/sqrt(s)
                % Convert to per-sample std dev using outputRate (Hz):
                % sigma ≈ ARW * sqrt(outputRate)
                noise = obj.ARW .* sqrt(obj.outputRate) .* randn(3,1);
            end

            noisyGyro = obj.w + b + noise;
        end
    end
end