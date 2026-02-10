classdef Uncertainty < handle
    properties
        variables 
        lhsSamples
        numRuns
    end
    
    methods
        function obj = Uncertainty(numRuns)
            obj.numRuns = numRuns;
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
        
        function generateLhsSamples(obj)
            numVars = length(obj.variables);
            % Generate base LHS samples in [0, 1]
            baseSamples = lhsdesign(obj.numRuns, numVars);
            obj.lhsSamples = zeros(obj.numRuns, numVars);
            
            for i = 1:numVars
                obj.lhsSamples(:, i) = obj.variables(i).ppf(baseSamples(:, i));
            end
        end
        
        function samples = getLhsSamples(obj)
             samples = obj.lhsSamples;
        end
    end
end