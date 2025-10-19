% Low Pass Filter, utilizing moving averages to filter data

classdef LowPassFilter < handle

    properties
        windowSize % Window size for moving average
        buffer % buffer of values to calculate moving average
        bufferIndex % Current index of buffer
        sumValue % Sum of values in window
        iteration % Iteration number of loop
    end
    
    methods
        % Filter Definition
        function obj = LowPassFilter(windowSize)
            
            obj.windowSize = windowSize;
            obj.buffer = zeros(1, windowSize);
            obj.bufferIndex = 1;
            obj.sumValue = 0;
            obj.iteration = 0;
        end

        % Return Filter Config
        function config = getConfig(obj)
            config.sumValue = obj.sumValue;
            config.iteration = obj.iteration;
        end

        % Iterative Filter Function
        function filteredValue = filter(obj,reading)

            oldReading = obj.buffer(obj.bufferIndex);
            obj.buffer(obj.bufferIndex) = reading;
            obj.bufferIndex = mod(obj.bufferIndex, obj.windowSize) + 1;

            obj.sumValue = obj.sumValue + reading - oldReading;
            
            obj.iteration = obj.iteration + 1;

            denom = min(obj.iteration, obj.windowSize);
            
            filteredValue = obj.sumValue / denom;

        end
      
        %Reset Function for member variables
        function reset(obj)
            obj.buffer = zeroes(1, obj.windowSize);
            obj.bufferIndex = 1;
            obj.sumValue = 0;
            obj.iteration = 0;
        end

    end
end