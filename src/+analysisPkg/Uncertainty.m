classdef Uncertainty < handle
    properties
        variables % need to figure out type hinting here
    end
    
    methods
        function obj = Uncertainty()
            obj.variables = [];
        end
        
        function addVariable(obj, name, nominal, deviation, distribution)
            newVar = analysisPkg.UncertaintyValue(name, nominal, deviation, distribution);
            if isempty(obj.variables)
                obj.variables = newVar;
            else
                obj.variables(end+1) = newVar;
            end
        end
        
        function lhsSamples = generateLhsSamples(obj, numRuns)
            arguments
                obj
                numRuns (1,1) double {mustBeNonnegative}
            end
            numVars = length(obj.variables);
            % Generate base LHS samples in [0, 1]
            baseSamples = lhsdesign(numRuns, numVars);
            lhsSamples = zeros(numRuns, numVars);
            
            for i = 1:numVars
                lhsSamples(:, i) = obj.variables(i).percentPointFunction(baseSamples(:, i));
            end
        end
    end
end