classdef Flight
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here

    properties (SetAccess = private)
        simTime;
        tStep;
        t;

        initState;
        currentState;
        stateHistory;
    end

    methods
        function obj = Flight()
            %UNTITLED2 Construct an instance of this class
            %   Detailed explanation goes here
        end

        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end