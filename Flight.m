classdef Flight
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here

    properties (SetAccess = private)
        tStep; % seconds (double)
        t; % seconds (double)
        runClosedLoop; % boolean
        calcXDots; % boolean
        runLinearModel; % boolean
        applyWind; % boolean
        useLatingHyperCubeSampling; % boolean
        numRuns; % integer (double)
        numStates = 13; 
        numInputs = 3;
        localSimTime; % seconds (double)
        simTime; % seconds (double)
        tSpan; % Vector of doubles
        gravity = 9.81 % m/(s^2)
        uSpan; % length of t span by 3 array of double
        
        initState;
        currentState;
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

        function data = runFlight(obj, Rocket, Environment)
            data(obj.numRuns) = struct();
            for runIndex = 1:obj.numRuns
                fprintf('Iterative LQR Monte Carlo Run %d out of %d...\n',runIndex,obj.numRuns)
                % Number of LQR intervals to simulate
                % currently each LQR interval is 1 second
                % NOTE: FSW will be switching LQR gains at a much faster rate (10 Hz)
                numIntervals = obj.simTime / obj.localSimTime;
                % Allocate log memory for current MC run
                tempMClog(numIntervals) = struct();

                if obj.get_runLinearModel()
                    tempMClinLog(numIntervals) = struct();
                end

                if obj.get_useLatinHyperCubeSampling()
                    mcParams = getLatinHyperCubeSample(obj.get_MCxlsx(), runIndex);
                else
                    mcParams = getRandomSample(obj.get_MCxlsx(), runIndex);
                end

                for lqrIndex =  1:numIntervals
                    fprintf('Current LQR config: %d \n', lqrIndex);

                    % Add Monte Carlo Parameters to input configuration
                    obj.set_windParam(mcParams.windParam);
                    Rocket.set_wet_mass(mcParams.wetMass);
                    Rocket.set_dry_mass(mcParams.dryMass);
                    % peak thrust?? Where is this used? config.peakThrust =
                    % mcParams.peakThrust
                    Rocket.set_burn_time(mcParams.burnTime);
                    Rocket.set_wet_inertias(mcParams.wetMOI);
                    Rocket.set_dry_inertias(mcParams.dryMOI);

                    % Get initial state for current LQR time interval

                end
            end
        end

    end
end
