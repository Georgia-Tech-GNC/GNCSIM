
classdef LowPassFilter

    properties
        smoothFactor % Smoothening Factor
        previousValue % Previous sensor reading
    end
    
    methods

        function obj = LowPassFilter(smoothFactor)
            obj.smoothFactor = smoothFactor;
            obj.previousValue = 0;
        end

        function factor = getSmoothFactor(obj)
            factor = obj.smoothFactor;
        end

        function filteredValue = filter(obj, reading)
            filteredValue = (reading * obj.smoothFactor) + (obj.previousValue * (1 - obj.smoothFactor));
            obj.previousValue = reading;
        end

    end
end