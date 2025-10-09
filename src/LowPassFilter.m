
classdef LowPassFilter < handle

    properties
        windowSize
        buffer
        bufferIndex
        sumValue
        iteration
    end
    
    methods
        function obj = LowPassFilter(windowSize)
            obj.windowSize = windowSize;
            obj.buffer = zeros(1, windowSize);
            obj.bufferIndex = 1;
            obj.sumValue = 0;
            obj.iteration = 0;
        end

        function config = getConfig(obj)
            config.sumValue = obj.sumValue;
            config.iteration = obj.iteration;
        end

        function filteredValue = filter(obj,reading)

            oldReading = obj.buffer(obj.bufferIndex);
            obj.buffer(obj.bufferIndex) = reading;
            obj.bufferIndex = mod(obj.bufferIndex, obj.windowSize) + 1;

            obj.sumValue = obj.sumValue + reading - oldReading;
            
            obj.iteration = obj.iteration + 1;

            denom = min(obj.iteration, obj.windowSize);
            
            filteredValue = obj.sumValue / denom;

        end
      

    end
end