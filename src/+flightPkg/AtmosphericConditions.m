classdef AtmosphericConditions
    properties
        Density (1,1) double
        SpeedOfSound (1,1) double
        Pressure (1,1) double
        Temperature (1,1) double
        WindVector (3,1) double = [0;0;0]
        Viscosity (1,1) double = 1.8e-5 % Optional default
    end
    
    methods
        function obj = AtmosphericConditions(rho, sos, P, T, wind)
            if nargin > 0
                obj.Density = rho;
                obj.SpeedOfSound = sos;
                obj.Pressure = P;
                obj.Temperature = T;
                obj.WindVector = [0;0;0];
            end
        end
    end
end
