classdef MonteCarlo

    properties (Access = private)
        
        PrivateProperty1
    end

    methods
        function obj = MonteCarlo(arg1, arg2)
            % obj = MyClassName(arg1, arg2) initializes the properties
            if nargin > 0
                obj.Property1 = arg1;
                obj.Property2 = arg2;
            end
        end
    end
end