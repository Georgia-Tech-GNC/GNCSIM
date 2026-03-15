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
        % Truth (latest provided state sample)
        w   % true angular rate (3x1) [rad/s]

        % Noise model parameters (all REQUIRED from JSON)
        bias         % constant bias (3x1) [rad/s]
        biasRW_sigma % bias random-walk sigma (3x1) [rad/s/sqrt(s)]
        biasRW       % current random-walk bias state (3x1) [rad/s]

        % Latency model (REQUIRED)
        latency_s
        latency_steps
        latency_buf
        latency_idx

        % Sample/hold
        heldMeasurement
        lastUpdateTime
    end

    methods
        function obj = Gyroscope(gyroParams)
            obj@navigationPkg.Sensor();

            % Required fields (assume present)
            obj.noiseCovariance   = gyroParams.noiseCovariance;
            obj.outputRate        = gyroParams.outputRate;
            obj.outputProbability = gyroParams.outputProbability;
            obj.observationMatrix = gyroParams.observationMatrix;

            obj.bias         = gyroParams.bias(:);
            obj.biasRW_sigma = gyroParams.biasRW_sigma(:);

            obj.latency_s = gyroParams.latency_s;

            % Derived timing
            obj.outputPeriod   = 1 / obj.outputRate;
            obj.lastOutputTime = 0.0;
            obj.lastUpdateTime = 0.0;
            obj.nextOutputTime = obj.outputPeriod;

            % States
            obj.w = zeros(3,1);
            obj.biasRW = zeros(3,1);
            obj.heldMeasurement = zeros(3,1);

            % Validate + initialize latency buffer
            obj.validateParams();
            obj.configureLatency();
        end

        function measurement = getMeasurement(obj, state, ~, currentTime) %#ok<INUSD>
            % Cache latest truth omega (state comes from Flight loop)
            obj.w = state(obj.idx.omega); % 3x1 true omega

            % First call: initialize to a sensible value (avoid zeros)
            if obj.lastUpdateTime == 0.0 && obj.lastOutputTime == 0.0
                y0 = obj.w + obj.bias;         % no noise on first output
                obj.initializeLatencyBuffer(y0);

                obj.heldMeasurement = obj.popLatency(); % output oldest (initialized to y0)
                obj.lastUpdateTime = currentTime;
                obj.lastOutputTime = currentTime;
                obj.nextOutputTime = currentTime + obj.outputPeriod;

                measurement = obj.heldMeasurement;
                return;
            end

            % Update at sensor rate; hold between sensor ticks
            if currentTime >= obj.nextOutputTime
                while currentTime >= obj.nextOutputTime
                    obj.stepOneSample(obj.nextOutputTime);
                    obj.lastOutputTime = obj.nextOutputTime;
                    obj.nextOutputTime = obj.nextOutputTime + obj.outputPeriod;
                end
            end

            measurement = obj.heldMeasurement;
        end
    end

    methods (Access = private)
        function validateParams(obj)
            if ~(isnumeric(obj.noiseCovariance) && all(size(obj.noiseCovariance) == [3 3]))
                error("Gyroscope:BadNoiseCov", "noiseCovariance must be a 3x3 matrix.");
            end
            % Cholesky will validate PSD in sampleWhiteNoise()

            if ~(isnumeric(obj.bias) && numel(obj.bias) == 3)
                error("Gyroscope:BadBias", "bias must be length-3.");
            end
            if ~(isnumeric(obj.biasRW_sigma) && numel(obj.biasRW_sigma) == 3)
                error("Gyroscope:BadBiasRW", "biasRW_sigma must be length-3.");
            end
            if ~(isnumeric(obj.latency_s) && isscalar(obj.latency_s) && isfinite(obj.latency_s) && obj.latency_s >= 0)
                error("Gyroscope:BadLatency", "latency_s must be a finite scalar >= 0.");
            end
            if ~(isnumeric(obj.outputRate) && isscalar(obj.outputRate) && obj.outputRate > 0)
                error("Gyroscope:BadRate", "outputRate must be > 0.");
            end
            if ~(isnumeric(obj.outputProbability) && isscalar(obj.outputProbability) && obj.outputProbability > 0 && obj.outputProbability <= 1)
                error("Gyroscope:BadProbability", "outputProbability must be in (0, 1].");
            end
        end

        function configureLatency(obj)
            obj.latency_steps = max(0, round(obj.latency_s / obj.outputPeriod));
            n = obj.latency_steps + 1;

            obj.latency_buf = zeros(3, n);
            obj.latency_idx = 1;
        end

        function initializeLatencyBuffer(obj, value)
            obj.latency_buf(:,:) = repmat(value(:), 1, size(obj.latency_buf, 2));
            obj.latency_idx = 1;
        end

        function stepOneSample(obj, tSample)
            dt = tSample - obj.lastUpdateTime;
            if ~isfinite(dt) || dt <= 0
                dt = obj.outputPeriod;
            end
            obj.lastUpdateTime = tSample;

            % Dropout (keep holding previous output)
            if obj.outputProbability < 1 && rand() > obj.outputProbability
                return;
            end

            % Pipeline (as requested):
            % raw truth -> add bias/noise -> buffer -> output delayed sample

            % 1) Bias drift random walk
            obj.biasRW = obj.biasRW + obj.biasRW_sigma .* sqrt(dt) .* randn(3,1);

            % 2) White noise
            noise = obj.sampleWhiteNoise();

            % 3) Raw measurement with bias + noise
            y_meas = obj.w + obj.bias + obj.biasRW + noise;

            % 4) Latency buffer (fixed delay applied to final measurement)
            obj.pushLatency(y_meas);
            obj.heldMeasurement = obj.popLatency();
        end

        function noise = sampleWhiteNoise(obj)
            % noise ~ N(0, R)
            [L,p] = chol(obj.noiseCovariance, 'lower');
            if p ~= 0
                error("Gyroscope:BadNoiseCov", "noiseCovariance must be symmetric PSD.");
            end
            noise = L * randn(3,1);
        end

        function pushLatency(obj, y_in)
            obj.latency_buf(:, obj.latency_idx) = y_in;

            % advance index (ring)
            n = size(obj.latency_buf, 2);
            obj.latency_idx = obj.latency_idx + 1;
            if obj.latency_idx > n
                obj.latency_idx = 1;
            end
        end

        function y_out = popLatency(obj)
            % latency_idx always points to the "oldest" sample after push+advance
            y_out = obj.latency_buf(:, obj.latency_idx);
        end
    end
end
