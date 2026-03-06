classdef UncertaintyValue < handle
    properties
        name string
        nominalValue (1,1) double
        deviation (1,1) double
        distribution (1,1) analysisPkg.DistributionType
    end
    
    methods
        function obj = UncertaintyValue(name, nominal, deviation, distribution)
            obj.name = name;
            obj.nominalValue = nominal;
            obj.deviation = deviation;
            obj.distribution = distribution;
        end
        
        function values = percentPointFunction(obj, probabilities)
            import analysisPkg.DistributionType
            
            switch obj.distribution
                case analysisPkg.DistributionType.GAUSSIAN
                    sigma = obj.deviation / 3;
                    values = norminv(probabilities, obj.nominalValue, sigma);
                case analysisPkg.DistributionType.UNIFORM
                    minVal = obj.nominalValue - obj.deviation;
                    maxVal = obj.nominalValue + obj.deviation;
                    values = minVal + probabilities .* (maxVal - minVal);
                otherwise
                    error('Unknown distribution type: %s', string(obj.distribution));
            end
        end
    end
end