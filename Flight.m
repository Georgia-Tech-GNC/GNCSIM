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
        useLatinHyperCubeSampling; % boolean
        numRuns; % integer (double)
        numStates = 13; 
        numInputs = 3;
        localSimTime; % seconds (double)
        simTime; % seconds (double)
        tSpan; % Vector of doubles
        gravity = 9.81 % m/(s^2)
        uSpan; % length of t span by 3 array of double
        MCxlsx; % filename of Excel sheet with Monte Carlo information
        
        initState;
        currentState;
    end

    methods
        function obj = Flight()
            %UNTITLED2 Construct an instance of this class
            %   Detailed explanation goes here
        end

        function obj= set_tStep(obj, t_step)
            if ~isdouble(t_step) || t_step < 0
                error('Time step must be a positive double.')
            else
                obj.tStep = t_step;
            end
        end

        function obj= set_runClosedLoop(obj, isClosedLoop)
            if ~islogical(isClosedLoop)
                error('runClosedLoop must be a logical value.')
            else
                obj.runClosedLoop = isClosedLoop;
            end
        end

        function obj= set_calcXDots(obj, calc_xdots)
            if ~islogical(calc_xdots)
                error('calcXdots must be a logical value.')
            else
                obj.calcXDots = calc_xdots;
            end
        end

        function obj= set_runLinearModel(obj, run_linmodel)
            if ~islogical(run_linmodel)
                error('runLinearModel must be a logical value.')
            else
                obj.runLinearModel = run_linmodel;
            end
        end

        function obj= set_applyWind(obj, apply_wind)
            if ~islogical(apply_wind)
                error('applyWind must be a logical value.')
            else
                obj.applyWind = apply_wind;
            end
        end

        function obj= set_useLatinHyperCubeSampling(obj, latin_sample)
            if ~islogical(latin_sample)
                error('useLatinHyperCubeSampling must be a logical value.')
            else
                obj.useLatinHyperCubeSampling = latin_sample;
            end
        end

        function obj= set_numRuns(obj, num_runs)
            if ~isdouble(num_runs) || num_runs < 1
                error('numRuns must be a positive double greater than 1.')
            else
                obj.numRuns = floor(num_runs);
            end
        end

        function obj= set_numStates(obj, num_states)
            if ~isdouble(num_states) || num_states < 1
                error('numStates must be a positive double greater than 1.')
            else
                obj.numRuns = floor(num_states);
            end
        end

        function obj= set_numInputs(obj, num_in)
            if ~isdouble(num_in) || num_in < 1
                error('numInputs must be a positive double greater than 1.')
            else
                obj.numInputs = floor(num_in);
            end
        end

        function obj= set_localSimTime(obj, lsimt)
            if ~isdouble(lsimt) || lsimt < 0
                error('localSimTime must be a positive double.')
            else
                obj.localSimTime = lsimt;
            end
        end

        function obj= set_simTime(obj, simt)
            if ~isdouble(simt) || simt < 0
                error('simTime must be a positive double.')
            else
                obj.simTime = simt;
            end
        end

        function obj= set_gravity(obj, g)
            if ~isdouble(g) || g < 0
                error('gravity must be a positive double.')
            else
                obj.gravity = g;
            end
        end

        function obj= set_initState(obj, x0)
            if ~isdouble(x0) || len(x0 ~= obj.numStates)
                error('initState must be a vector of doubles of length numStates.')
            else
                obj.initState = x0;
            end
        end

        function obj= set_tSpan(obj)
            obj.tSpan = 0:obj.tStep:obj.simTime;
        end

        function obj= set_uSpan(obj, u_span)
            [r, c] = size(u_span);
            if ~isdouble(u_span) || r ~= length(obj.tSpan) || c ~= 3
                error('uSpan must be a tSpan x 3 array of values representing arbitrary external moments on the rocket');
            else
                obj.uSpan = u_span;
            end
        end

        function obj=set_MCxlsx(obj, fn)
            if ~ischar(fn)
                error('MCxlsx must be a filename in the form of a char array')
            else
                obj.MCxlsx = fn;
            end
        end

        function tstep= get_tStep(obj)
            tstep = obj.tStep;
        end

        function rcl= get_runClosedLoop(obj)
            rcl = obj.runClosedLoop;
        end

        function cxd = get_calcXdots(obj)
            cxd = obj.calcXDots;
        end

        function rlm = get_runLinearModel(obj)
            rlm = obj.runLinearModel;
        end

        function aw = get_applyWind(obj)
            aw = obj.applyWind;
        end

        function ulhcs = get_useLatinHyperCubeSampling(obj)
            ulhcs = obj.useLatinHyperCubeSampling;
        end

        function nr = get_numRuns(obj)
            nr = obj.numRuns;
        end

        function ns = get_numStates(obj)
            ns = obj.numStates;
        end
        
        function ni = get_numInputs(obj)
            ni = obj.numInputs;
        end

        function lst = get_localSimTime(obj)
            lst = obj.localSimTime;
        end

        function st = get_simTime(obj)
            st = obj.simTime;
        end

        function ts = get_tSpan(obj)
            ts = obj.tSpan;
        end

        function g = get_gravity(obj)
            g = obj.gravity;
        end

        function us = get_uSpan(obj)
            us = obj.uSpan;
        end

        function is = get_initState(obj)
            is = obj.initState;
        end

        function mcParams = getRandomSample(xlsxFile,runIndex)
            % getRandomSample
            xlsxFile = readcell(xlsxFile);
    
            gustAmplitude = random(makedist(xlsxFile{2*runIndex,4},'mu',xlsxFile{2*runIndex,5},'sigma',xlsxFile{2*runIndex,6}/3));
            gustLength = random(makedist(xlsxFile{1+2*runIndex,4},'mu',xlsxFile{1+2*runIndex,5},'sigma',xlsxFile{1+2*runIndex,6}/3));
            windParam = [gustAmplitude gustLength];
    
            % INSERT NEW MONTE CARLO PARAM (READ FROM EXCEL SHEET) HERE
            wetMass = 38.87;% update to read from excel sheet
            dryMass = 30.96; % update to read from excel sheet
            peakThrust = 2262; % update to read from excel sheet
            burnTime = 13.1; % update to read from excel sheet
            % thrustCurve = generate_thrust_curve(peakThrust, burnTime);
            wetRollMOI = 0.2109; % update to read from excel sheet
            wetPitchMOI = 21.1212; % update to read from excel sheet
            wetYawMOI = 21.1202; % update to read from excel sheet
            dryRollMOI = 0.2036; % update to read from excel sheet
            dryPitchMOI = 18.99; % update to read from excel sheet
            dryYawMOI = 18.988; % update to read from excel sheet
    
            % UPDATE OUTPUT STRUCT
            mcParams.windParam = windParam;
            mcParams.wetMass = wetMass;
            mcParams.dryMass = dryMass;
            mcParams.peakThrust = peakThrust;
            mcParams.burnTime = burnTime;
            mcParams.thrustCurve = thrustCurve;
            mcParams.wetMOI = [wetRollMOI 0 0;
                                0 wetPitchMOI 0;
                                0 0 wetYawMOI];
            mcParams.dryMOI = [dryRollMOI 0 0;
                                0 dryPitchMOI 0;
                                0 0 dryYawMOI];
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
                    % Pick up the work here %

                end
            end
        end

    end
end
