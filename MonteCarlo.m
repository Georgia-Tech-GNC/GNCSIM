classdef MonteCarlo

    properties (Access = private)
        % Monte Carlo Hyperparameters
        runGaussian; % boolean
        runSweep; % boolean

        % Rocket Parameters - Mean Value
        wetMass;
        dryMass;
        burnTime;
        thrustCurve; % 2d array
        wetMOI; % 3x3 matrix
        dryMOI; % 3x3 matrix

        % Rocket Parameters - 3 Sigma
        wetMass_3sigma;
        dryMass_3sigma;
        burnTime_3sigma;
        thrustCurve_3sigma; % scalar
        wetMOI_3sigma; % 1x3 vector
        dryMOI_3sigma; % 1x3 vector

        % Wind Parameters
        windIndex; % mean values and 3sigma are in WindParameters.xslx

        % Actuator Model Parameters - Mean Value
        % Actuator Model Parameters - 3 Sigma

    end

    methods
        % Constructor for the class
        function obj = MonteCarlo()
        end

        %Setters
        function obj = set_numRuns(obj, value)
            validateattributes(value, {'numeric'}, {'nonnegative', 'scalar'}, 'set_numRuns', 'numRuns');
            obj.numRuns = value;
        end
        
        function obj = set_runGaussian(obj, value)
            validateattributes(value, {'logical'}, {'scalar'}, 'set_runGaussian', 'runGaussian');
            obj.runGaussian = value;
        end
        
        function obj = set_runSweep(obj, value)
            validateattributes(value, {'logical'}, {'scalar'}, 'set_runSweep', 'runSweep');
            obj.runSweep = value;
        end

        function obj = set.wetMass(obj, value)
            validateattributes(value, {'numeric'}, {'nonnegative', 'scalar'}, 'set_wetMass', 'wetMass');
            obj.wetMass = value;
        end

        function obj = set.dryMass(obj, value)
            validateattributes(value, {'numeric'}, {'nonnegative', 'scalar'}, 'set_dryMass', 'dryMass');
            obj.dryMass = value;
        end

        function obj = set.burnTime(obj, value)
            validateattributes(value, {'numeric'}, {'nonnegative', 'scalar'}, 'set_burnTime', 'burnTime');
            obj.burnTime = value;
        end

        function obj = set.thrustCurve(obj, value)
            validateattributes(value, {'numeric'}, {'2d'}, 'set_thrustCurve', 'thrustCurve');
            obj.thrustCurve = value;
        end

        function obj = set.wetMOI(obj, value)
            validateattributes(value, {'numeric'}, {'size', [3, 3]}, 'set_wetMOI', 'wetMOI');
            obj.wetMOI = value;
        end

        function obj = set.dryMOI(obj, value)
            validateattributes(value, {'numeric'}, {'size', [3, 3]}, 'set_dryMOI', 'dryMOI');
            obj.dryMOI = value;
        end

        function obj = set.wetMass_3sigma(obj, value)
            validateattributes(value, {'numeric'}, {'scalar', 'nonnegative'}, 'set.wetMass_3sigma', 'wetMass_3sigma');
            obj.wetMass_3sigma = value;
        end

        function obj = set.dryMass_3sigma(obj, value)
            validateattributes(value, {'numeric'}, {'scalar', 'nonnegative'}, 'set.dryMass_3sigma', 'dryMass_3sigma');
            obj.dryMass_3sigma = value;
        end


        function obj = set.burnTime_3sigma(obj, value)
            validateattributes(value, {'numeric'}, {'scalar', 'positive'}, 'set.burnTime_3sigma', 'burnTime_3sigma');
            obj.burnTime_3sigma = value;
        end

        function obj = set.thrustCurve_3sigma(obj, value)
            validateattributes(value, {'numeric'}, {'scalar'}, 'set.thrustCurve_3sigma', 'thrustCurve_3sigma');
            obj.thrustCurve_3sigma = value;
        end

        function obj = set.wetMOI_3sigma(obj, value)
            validateattributes(value, {'numeric'}, {'size', [1, 3]}, 'set.wetMOI_3sigma', 'wetMOI_3sigma');
            obj.wetMOI_3sigma = value;
        end

        function obj = set.dryMOI_3sigma(obj, value)
            validateattributes(value, {'numeric'}, {'size', [1, 3]}, 'set.dryMOI_3sigma', 'dryMOI_3sigma');
            obj.dryMOI_3sigma = value;
        end

        function obj = set_windIndex(obj, value)
            validateattributes(value, {'numeric'}, {'integer', 'positive'}, 'set_windIndex', 'windIndex');
            obj.windIndex = value;
        end

        function logMonteCarlo = runMonteCarlo(obj, numRuns, FlightObj, RocketObj, EnvironmentObj, AerodynamicsObj, TrimPointObj)
            
            logMonteCarlo = 
            for i = 1:numRuns
                log = runFlight(obj, Rocket, Environment, Aerodynamics, TrimPointObj);
            end
        end

    end

    methods (Static)
        function randomValue = generateGaussianRandom(meanValue, stdValue)
            % generateGaussianRandom - Generate a Gaussian random number
            %
            % Inputs:
            %   meanValue - The mean value (expected value)
            %   stdValue  - The standard deviation (3sigma)
            %
            % Outputs:
            %   randomValue - A single Gaussian random number
            
            % Calculate the standard deviation from the 3-sigma value
            sigma = stdValue / 3;
            
            % Generate a Gaussian random number
            randomValue = meanValue + sigma * randn();
        end
    end
end
