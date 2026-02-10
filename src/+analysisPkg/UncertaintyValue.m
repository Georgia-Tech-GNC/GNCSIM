classdef UncertaintyValue < handle
    properties
        name
        nominalValue
        deviation 
        distribution 
    end
    
    methods
        function obj = UncertaintyValue(name, nominal, deviation, distribution)
            obj.name = name;
            obj.nominalValue = nominal;
            obj.deviation = deviation;
            obj.distribution = distribution;
        end
        
        function values = ppf(obj, probabilities)
            if strcmpi(obj.distribution, 'normal')
                sigma = obj.deviation / 3;
                values = norminv(probabilities, obj.nominalValue, sigma);
            else
                minVal = obj.nominalValue - obj.deviation;
                maxVal = obj.nominalValue + obj.deviation;
                values = minVal + probabilities .* (maxVal - minVal);
            end
        end
    end
end