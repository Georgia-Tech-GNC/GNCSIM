classdef MonteCarlo

    properties (Access = private)
        % Monte Carlo Hyperparameters
        numRuns; 
        runGaussian; % boolean
        runSweep; %boolean

        % Rocket Parameters
            % Mean Value
        wetMass;
        dryMass;
        peakThrust;
        burnTime;
        thrustCurve;
        wetMOI; % 3x3 matrix
        dryMOI; % 3x3 matrix
            % Standard Deviation
        wetMass_std;
        dryMass_std;
        peakThrust_std;
        burnTime_std;
        thrustCurve_std;
        wetMOI_std; % 3x3 matrix
        dryMOI_std; % 3x3 matrix

        % Wind Parameters
            % Mean Value
            % Standard Deviation

        % Actuator Model Parameters
            % Mean Value
            % Standard Deviation

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

        function obj = set.peakThrust(obj, value)
            validateattributes(value, {'numeric'}, {'nonnegative', 'scalar'}, 'set_peakThrust', 'peakThrust');
            obj.peakThrust = value;
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

        % Gaussian Noise Function
        % Input: Parameter mean value and standard deviation
        % Output: Random gaussian value

    end
end
