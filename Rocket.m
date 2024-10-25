classdef Rocket
    %Summary of this class goes here
    %   Detailed explanation goes here

    properties (SetAccess = private)
        wetMass;
        dryMass;
        currentMass;
        wetInertias;
        dryInertias;
        currentInertias;
        initialCG; %distance from nosecone to CG with loaded motor
        finalCG; %distance from nosecone to CG with empty motor
        currentCG;
        CP;
        length;
        diameter;
        radius;


        burnTime;
        thrustCurve;
    end

    methods
        function obj = Rocket()
            %Construct an instance of this class
            %   Detailed explanation goes here
        end

        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end