classdef MonteCarlo < handle

    properties (Access = private)
        % Monte Carlo Hyperparameters
        runGaussianLHS; % boolean (LHS = Latin Hypercube Sampling)
        runUniformLHS; % boolean
        runCustomLHS;
        numRuns; % integer
        %numVariables; % integer

        lhsSamples;

        % Rocket Parameters - 3 Sigma
        wetMass_3sigma;
        dryMass_3sigma;
        wetRollMOI_3sigma; 
        wetPitchMOI_3sigma;
        wetYawMOI_3sigma;
        dryRollMOI_3sigma;
        dryPitchMOI_3sigma;
        dryYawMOI_3sigma;
        initialCG_3sigma; 
        finalCG_3sigma;
        burnTime_3sigma;
        thrustCurve_3sigma;

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

        % function obj = set_numVariables(obj, value)
        %     validateattributes(value, {'numeric'}, {'nonnegative', 'scalar'}, 'set_numVariables', 'numVariables');
        %     obj.numVariables = value;
        % end
        
        function obj = set_runGaussianLHS(obj, value)
            validateattributes(value, {'logical'}, {'scalar'}, 'set_runGaussianLHS', 'runGaussianLHS');
            obj.runGaussianLHS = value;
        end

        function obj = set_runCustomLHS(obj, value)
            validateattributes(value, {'logical'}, {'scalar'}, 'set_runCustomLHS', 'runCustomLHS');
            obj.runGaussianLHS = value;
        end

        % function obj = set_runUniformLHS(obj, value)
        %     validateattributes(value, {'logical'}, {'scalar'}, 'set_runSweep', 'runUniformLHS');
        %     obj.runUniformLHS = value;
        % end

        function obj = set_wetMass_3sigma(obj, value)
            validateattributes(value, {'numeric'}, {'scalar', 'nonnegative'}, 'set_wetMass_3sigma', 'wetMass_3sigma');
            obj.wetMass_3sigma = value;
        end

        function obj = set_dryMass_3sigma(obj, value)
            validateattributes(value, {'numeric'}, {'scalar', 'nonnegative'}, 'set_dryMass_3sigma', 'dryMass_3sigma');
            obj.dryMass_3sigma = value;
        end

        function obj = set_wetRollMOI_3sigma(obj, value)
            validateattributes(value, {'numeric'}, {'scalar', 'nonnegative'}, 'set_wetRollMOI_3sigma', 'wetRollMOI_3sigma');
            obj.wetRollMOI_3sigma = value;
        end
        
        function obj = set_wetPitchMOI_3sigma(obj, value)
            validateattributes(value, {'numeric'}, {'scalar', 'nonnegative'}, 'set_wetPitchMOI_3sigma', 'wetPitchMOI_3sigma');
            obj.wetPitchMOI_3sigma = value;
        end
        
        function obj = set_wetYawMOI_3sigma(obj, value)
            validateattributes(value, {'numeric'}, {'scalar', 'nonnegative'}, 'set_wetYawMOI_3sigma', 'wetYawMOI_3sigma');
            obj.wetYawMOI_3sigma = value;
        end
        
        function obj = set_dryRollMOI_3sigma(obj, value)
            validateattributes(value, {'numeric'}, {'scalar', 'nonnegative'}, 'set_dryRollMOI_3sigma', 'dryRollMOI_3sigma');
            obj.dryRollMOI_3sigma = value;
        end
        
        function obj = set_dryPitchMOI_3sigma(obj, value)
            validateattributes(value, {'numeric'}, {'scalar', 'nonnegative'}, 'set_dryPitchMOI_3sigma', 'dryPitchMOI_3sigma');
            obj.dryPitchMOI_3sigma = value;
        end
        
        function obj = set_dryYawMOI_3sigma(obj, value)
            validateattributes(value, {'numeric'}, {'scalar', 'nonnegative'}, 'set_dryYawMOI_3sigma', 'dryYawMOI_3sigma');
            obj.dryYawMOI_3sigma = value;
        end

        function obj = set_initialCG_3sigma(obj, value)
            validateattributes(value, {'numeric'}, {'scalar', 'nonnegative'}, 'set_initialCG_3sigma', 'initialCG_3sigma');
            obj.initialCG_3sigma = value;
        end
        
        function obj = set_finalCG_3sigma(obj, value)
            validateattributes(value, {'numeric'}, {'scalar', 'nonnegative'}, 'set_finalCG_3sigma', 'finalCG_3sigma');
            obj.finalCG_3sigma = value;
        end

        function obj = set_burnTime_3sigma(obj, value)
            validateattributes(value, {'numeric'}, {'scalar', 'nonnegative'}, 'set_burnTime_3sigma', 'burnTime_3sigma');
            obj.burnTime_3sigma = value;
        end

        function obj = set_thrustCurve_3sigma(obj, value)
            validateattributes(value, {'numeric'}, {'scalar', 'nonnegative'}, 'set_thrustCurve_3sigma', 'thrustCurve_3sigma');
            obj.thrustCurve_3sigma = value;
        end

        function obj = set_windIndex(obj, value)
            validateattributes(value, {'numeric'}, {'integer', 'positive'}, 'set_windIndex', 'windIndex');
            obj.windIndex = value;
        end

        function obj = set_lhsSamples(obj, value)
            validateattributes(value, {'numeric'}, {'2d', 'nonempty'}, 'set_lhsSamples', 'lhsSamples');
            obj.lhsSamples = value;
        end
        
        % Getters
        function lhsSamples = get_lhsSamples(obj)
            if isempty(obj.lhsSamples)
                error('The lhsSamples property has not been initialized.');
            end
            lhsSamples = obj.lhsSamples;
        end

        % Function that runs MonteCarlo for numRuns
        function logMonteCarlo = runMonteCarlo(obj, FlightObj, RocketObj, EnvironmentObj, AerodynamicsObj, TrimPointObj)
            % Setting up LHS parameters  
            if obj.runGaussianLHS
                thrustCurve = RocketObj.get_thrustCurve();
                thrustCurve_mu = thrustCurve(:,2).';
                wetMOI = diag(RocketObj.get_wetMOI).';
                dryMOI = diag(RocketObj.get_dryMOI).';

                mu = [
                    RocketObj.get_wetMass(), ...           % Wet mass (kg)
                    RocketObj.get_dryMass(), ...           % Dry mass (kg)
                    wetMOI, ...
                    dryMOI, ...
                    RocketObj.get_initialCG(), ...         % Initial CG (m)
                    RocketObj.get_finalCG(), ...           % Final CG (m)
                    RocketObj.get_burnTime(), ...          % Burn time (s)
                    thrustCurve_mu                            % Flattened thrust curve (s, N)
                    ];

                sigma = [
                    obj.wetMass_3sigma, ...
                    obj.dryMass_3sigma, ...
                    obj.wetRollMOI_3sigma, ... 
                    obj.wetPitchMOI_3sigma, ...
                    obj.wetYawMOI_3sigma, ...
                    obj.dryRollMOI_3sigma, ...
                    obj.dryPitchMOI_3sigma, ...
                    obj.dryYawMOI_3sigma, ...
                    obj.initialCG_3sigma, ...
                    obj.finalCG_3sigma, ...
                    obj.burnTime_3sigma, ...
                    obj.thrustCurve_3sigma
                    ];

                sigma = (sigma ./ 3).^2; % Converting from 3sigma to variance.

                repetitions = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, length(RocketObj.thrustCurve(:,2))];
                sigma = diag(repelem(sigma, repetitions)); % Creating the covariance matrix for mu

                obj.lhsSamples = lhsnorm(mu, sigma, obj.numRuns);

            % elseif obj.runUniformLHS
            %     lhsSamples = lhsdesign(obj.numRuns, obj.numVariables);
            %     variable1 = 0 + (10 - 0) * lhsSamples(:, 1); % Uniform distribution

            elseif obj.runCustomLHS
                if isempty(obj.lhsSamples)
                    error("Please set lhsSamples with a correct Latin Hypercube Sample matrix.")
                end
            else
                error('Either runGaussianLHS, runUniformLHS or runCustomLHS must be set to true.')
            end
            
            % Preallocate struct array
            logMonteCarlo(obj.numRuns) = struct('loggedData', []);

            parpool;
            tic;

            % Parallel loop
            parfor i = 1:obj.numRuns
                % Making copy of objects to avoid broadcasting issues
                RocketObjIter = RocketObj; 
                FlightObjCopy = FlightObj; 
                objCopy = obj;  
                thrustCurveCopy = thrustCurve;

                % Assign random variables to RocketObjIter
                RocketObjIter.set_wetMass(objCopy.lhsSamples(i,1));
                RocketObjIter.set_dryMass(objCopy.lhsSamples(i,2));
                RocketObjIter.set_initialCG(objCopy.lhsSamples(i,9)); 
                RocketObjIter.set_finalCG(objCopy.lhsSamples(i,10)); 
                
                wetRollMOI = objCopy.lhsSamples(i,3); 
                wetPitchMOI = objCopy.lhsSamples(i,4); 
                wetYawMOI = objCopy.lhsSamples(i,5); 
                dryRollMOI = objCopy.lhsSamples(i,6);
                dryPitchMOI = objCopy.lhsSamples(i,7);
                dryYawMOI = objCopy.lhsSamples(i,8);
                
                RocketObjIter.set_wetMOI([wetRollMOI 0 0; 0 wetPitchMOI 0; 0 0 wetYawMOI]);
                RocketObjIter.set_dryMOI([dryRollMOI 0 0; 0 dryPitchMOI 0; 0 0 dryYawMOI]);
                RocketObjIter.set_burnTime(objCopy.lhsSamples(i,11));
                thrustCurveCopy(:,2) = objCopy.lhsSamples(i,12:end); 
                RocketObjIter.set_thrustCurve(thrustCurveCopy);
            
                % Run the flight simulation
                log = FlightObjCopy.runFlight(RocketObjIter, EnvironmentObj, AerodynamicsObj, TrimPointObj);
            
                % Store the results
                logMonteCarlo(i).loggedData = log;
            end

            timeFor = toc;
            fprintf('Elapsed time: %.2f seconds for %d iterations\n', timeFor, obj.numRuns);
        end

        % Save all parameters to a CSV file
        function saveToCSV(obj, filename)
            % Collect all properties into a structure
            paramStruct = struct( ...
                'wetMass_3sigma', obj.wetMass_3sigma, ...
                'dryMass_3sigma', obj.dryMass_3sigma, ...
                'wetRollMOI_3sigma', obj.wetRollMOI_3sigma, ...
                'wetPitchMOI_3sigma', obj.wetPitchMOI_3sigma, ...
                'wetYawMOI_3sigma', obj.wetYawMOI_3sigma, ...
                'dryRollMOI_3sigma', obj.dryRollMOI_3sigma, ...
                'dryPitchMOI_3sigma', obj.dryPitchMOI_3sigma, ...
                'dryYawMOI_3sigma', obj.dryYawMOI_3sigma, ...
                'initialCG_3sigma', obj.initialCG_3sigma, ...
                'finalCG_3sigma', obj.finalCG_3sigma, ...
                'burnTime_3sigma', obj.burnTime_3sigma, ...
                'thrustCurve_3sigma', obj.thrustCurve_3sigma);
    
            % Convert structure to cell array for CSV saving
            paramNames = fieldnames(paramStruct);
            paramValues = struct2cell(paramStruct);
    
            % Prepare the table for saving
            data = [paramNames, paramValues];
            writecell(data, filename);
            fprintf('Parameters saved to %s\n', filename);
        end
    
        % Load parameters from a CSV file
        function obj = loadFromCSV(obj, filename)
            try
                % Attempt to read the CSV file
                data = readcell(filename);
            catch ME
                % Check if the error is due to the file not being found
                if strcmp(ME.identifier, 'MATLAB:readcell:FileNotFound')
                    error('File "%s" not found. Please provide a valid file path.', filename);
                else
                    % Re-throw the error if it's a different issue
                    rethrow(ME);
                end
            end
        
            % Update class properties based on CSV data
            for i = 1:size(data, 1)
                fieldName = data{i, 1};
                fieldValue = data{i, 2};
        
                % Update property only if it exists in the class
                if isprop(obj, fieldName)
                    obj.(fieldName) = fieldValue;
                else
                    warning('Unknown property "%s" in file. Skipping.', fieldName);
                end
            end
        
            fprintf('Parameters loaded from %s\n', filename);
        end

        function plotLHSSamples(obj)
            % Extract the number of samples and variables
            [~, numVariables] = size(obj.lhsSamples);
    
            % Create a figure for the plots
            figure;
    
            % Plot all variable samples
            for i = 1:numVariables
                % Plot histogram for each variable
                subplot(ceil(numVariables / 3), 3, i); % Create a grid of subplots
                histogram(obj.lhsSamples(:, i), 20); % 20 bins for the histogram
                title(sprintf('Variable %d', i));
                xlabel('Value');
                ylabel('Frequency');
            end
    
            % Adjust layout
            sgtitle('LHS Samples Distribution'); % Add a title for the figure
        end

    end
end
