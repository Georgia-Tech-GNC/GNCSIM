classdef SimpleNavigation < handle
    % SimpleNavigation  Simple discrete-time navigation + roll controller
    %   No EKF. Adds IMU noise + bias, low-pass filters IMU rate, delays it,
    %   and produces a PID airframe torque command with clamping, anti-windup,
    %   and wheel-speed torque derating.
    %
    %   Public usage:
    %     nav = navigationPkg.SimpleNavigation(params);
    %     estState = nav.getStateEstimate(state, stateDot, time, log);
    %
    %   The 'params' struct may contain fields below; defaults are provided.

    properties
        idx
        biasVector
        noiseSigma
        alpha

        prevStateEstimate = 0;
        gyroscope
        accelerometer

        gyroRawData
        accelRawData
        
    end

    methods
        function obj = SimpleNavigation(dt, gyroscope, accelerometer)
            obj.idx = StateIndicies;

            % Adjust values if needed
            obj.noiseSigma = 0;
            obj.biasVector = zeros(14, 1);
            obj.biasVector(obj.idx.p) = 0.005;
            % Optional argument
            alpha = 0;

            obj.gyroscope = gyroscope;
            obj.accelerometer = accelerometer;
        end

        function state = getStateEstimate(obj, state, stateDot, time, log)
            noise = randn(size(state)) * obj.noiseSigma;
            obj.gyroRawData = obj.gyroscope.getMeasurement(state, stateDot, time);
            
            measuredState = obj.gyroRawData + noise + obj.biasVector;
            
            if time == 0
                filteredState = measuredState;
            else
            % FILTERING (Low Pass Filter), may need to adjust to moving
            % average if desired
            % Formula: y[k] = alpha * x[k] + (1-alpha) * y[k-1]
                filteredState = measuredState - prevStateEstimate; 

            
            prevStateEstimate = filteredState;
        end

        %  function filteredValue = filter(obj,reading)
        % 
        %     oldReading = obj.buffer(obj.bufferIndex);
        %     obj.buffer(obj.bufferIndex) = reading;
        %     obj.bufferIndex = mod(obj.bufferIndex, obj.windowSize) + 1;
        % 
        %     obj.sumValue = obj.sumValue + reading - oldReading;
        % 
        %     obj.iteration = obj.iteration + 1;
        % 
        %     denom = min(obj.iteration, obj.windowSize);
        % 
        %     filteredValue = obj.sumValue / denom;
        % 
        % end

    end
end
