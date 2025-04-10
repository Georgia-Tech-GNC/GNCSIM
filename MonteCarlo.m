% Main Authors: Karsten Caillet 
classdef MonteCarlo < handle

    properties (Access = private)
        % Monte Carlo Simulation Hyperparameters and variables
        runGaussianLHS;  % boolean (Use Gaussian LHS)
        runUniformLHS;   % boolean (Use Uniform LHS)
        runCustomLHS;    % boolean (Use user-provided LHS)
        numRuns;         % integer
        odeSolverMaxTime; 
        lhsSamples;      % stores the actual NxM LHS sample matrix

        % Monte Carlo Pass/Fail parameters
        evaluate_checkTermination = false;
        evaluate_checkRoll = false;
        evaluate_checkPitch = false;
        evaluate_checkYaw = false;
        evaluate_checkAoA = false;
        evaluate_rollRange; 
        evaluate_pitchRange;
        evaluate_yawRange; 
        evaluate_AoARange;

        % Rocket Parameters
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
    end

    methods
        % Constructor for the class
        function obj = MonteCarlo()
        end

        % ---------------------- Setters ----------------------
        function obj = set_numRuns(obj, value)
            validateattributes(value, {'numeric'}, {'nonnegative', 'scalar'}, 'set_numRuns', 'numRuns');
            obj.numRuns = value;
        end
        
        function obj = set_runGaussianLHS(obj, value)
            validateattributes(value, {'logical'}, {'scalar'}, 'set_runGaussianLHS', 'runGaussianLHS');
            obj.runGaussianLHS = value;
        end

        function obj = set_runCustomLHS(obj, value)
            validateattributes(value, {'logical'}, {'scalar'}, 'set_runCustomLHS', 'runCustomLHS');
            obj.runCustomLHS = value;
        end

        function obj = set_runUniformLHS(obj, value)
            validateattributes(value, {'logical'}, {'scalar'}, 'set_runUniformLHS', 'runUniformLHS');
            obj.runUniformLHS = value;
        end

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

        function obj = set_lhsSamples(obj, value)
            validateattributes(value, {'numeric'}, {'2d', 'nonempty'}, 'set_lhsSamples', 'lhsSamples');
            obj.lhsSamples = value;
        end

        function obj = set_odeSolverMaxTime(obj, value)
            validateattributes(value, {'numeric'}, {'nonnegative', 'scalar'}, mfilename, 'odeSolverMaxTime');
            obj.odeSolverMaxTime = value;
        end

        function obj = set_evaluate_checkTermination(obj, value)
            validateattributes(value, {'logical'}, {'scalar'}, 'set_evaluate_checkTermination', 'evaluate_checkTermination');
            obj.evaluate_checkTermination = value;
        end
        
        function obj = set_evaluate_checkRoll(obj, value)
            validateattributes(value, {'logical'}, {'scalar'}, 'set_evaluate_checkRoll', 'evaluate_checkRoll');
            obj.evaluate_checkRoll = value;
        end
        
        function obj = set_evaluate_checkPitch(obj, value)
            validateattributes(value, {'logical'}, {'scalar'}, 'set_evaluate_checkPitch', 'evaluate_checkPitch');
            obj.evaluate_checkPitch = value;
        end

        function obj = set_evaluate_checkYaw(obj, value)
            validateattributes(value, {'logical'}, {'scalar'}, 'set_evaluate_checkYaw', 'evaluate_checkYaw');
            obj.evaluate_checkYaw = value;
        end

        function obj = set_evaluate_checkAoA(obj, value)
            validateattributes(value, {'logical'}, {'scalar'}, 'set_evaluate_checkAoA', 'evaluate_checkAoA');
            obj.evaluate_checkAoA = value;
        end
        
        function obj = set_evaluate_rollRange(obj, value)
            validateattributes(value, {'cell'}, {}, 'set_evaluate_rollRange', 'evaluate_rollRange');
            obj.evaluate_rollRange = value;
        end

        function obj = set_evaluate_pitchRange(obj, value)
            validateattributes(value, {'cell'}, {}, 'set_evaluate_pitchRange', 'evaluate_pitchRange');
            obj.evaluate_pitchRange = value;
        end

        function obj = set_evaluate_yawRange(obj, value)
            validateattributes(value, {'cell'}, {}, 'set_evaluate_yawRange', 'evaluate_yawRange');
            obj.evaluate_yawRange = value;
        end

        function obj = set_evaluate_AoARange(obj, value)
            validateattributes(value, {'cell'}, {}, 'set_evaluate_AoARange', 'evaluate_AoARange');
            obj.evaluate_AoARange = value;
        end
        
        % ---------------------- Getters ----------------------
        function lhsSamples = get_lhsSamples(obj)
            if isempty(obj.lhsSamples)
                error('The lhsSamples property has not been initialized.');
            end
            lhsSamples = obj.lhsSamples;
        end
        
        function generateLhsSamples(obj, RocketObj)
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
                    thrustCurve_mu                         % Flattened thrust curve (s, N)
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
            elseif obj.runUniformLHS
                % Example: Uniform LHS generation (user must define bounds)
                % For demonstration, suppose the dimension is the same as "mu" above:
                nVars = 11 + length(RocketObj.get_thrustCurve(:,2)); 
                % Generate an NxM LHS in [0,1], then you can scale it as needed
                design = lhsdesign(obj.numRuns, nVars);
                % ... scale 'design' to the relevant ranges ...
                % Here, just store it so you can see how you'd do your own uniform scaling
                obj.lhsSamples = design;  
            elseif obj.runCustomLHS
                if isempty(obj.lhsSamples)
                    error("Please set lhsSamples with a correct Latin Hypercube Sample matrix.")
                end
            else
                error('Either runGaussianLHS, runUniformLHS or runCustomLHS must be set to true.')
            end
        end

        % Function that runs MonteCarlo for numRuns
        function logMonteCarlo = runMonteCarlo(obj, FlightObj, RocketObj, EnvironmentObj, AerodynamicsObj, TrimPointObj)
            FlightObj.set_odeSolverMaxTime(obj.odeSolverMaxTime);
            thrustCurve = RocketObj.get_thrustCurve();
            
            % Preallocate struct array
            logMonteCarlo(obj.numRuns) = struct('loggedData', [], 'pass', []);

            % Check for an existing parallel pool and start one if necessary.
            poolobj = gcp('nocreate'); % Get current pool without creating a new one.
            if isempty(poolobj)
                parpool;  % Start a new pool if one does not exist.
            end
            
            tic;

            % Parallel loop
            parfor i = 1:size(obj.lhsSamples, 1)
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
                
                wetRollMOI = abs(objCopy.lhsSamples(i,3)); 
                wetPitchMOI = abs(objCopy.lhsSamples(i,4)); 
                wetYawMOI = abs(objCopy.lhsSamples(i,5)); 
                dryRollMOI = abs(objCopy.lhsSamples(i,6));
                dryPitchMOI = abs(objCopy.lhsSamples(i,7));
                dryYawMOI = abs(objCopy.lhsSamples(i,8));
                
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
            fprintf('Elapsed time: %.2f seconds for %d iterations\n', timeFor, size(obj.lhsSamples, 1));
            save("logMonteCarlo")
        end

        function [logMonteCarlo, performance] = evaluateMonteCarlo(obj, logMonteCarlo)
            tic;
            numSims = length(logMonteCarlo);
            
            % Setting all pass to true by default
            for i = 1:numSims
                logMonteCarlo(i).pass = true;
            end

            % Check if ode solver got terminated for excessive solving time
            if obj.evaluate_checkTermination
                for i = 1:numSims
                    if logMonteCarlo(i).loggedData.loggedData.termination_flag == true
                        logMonteCarlo(i).pass = false;
                        fprintf('Index %d: termination_flag is true, setting pass to false.\n', i);
                    end
                end
            end

            % Check if roll is within bounds
            if obj.evaluate_checkRoll
                for i = 1:numSims
                    if logMonteCarlo(i).pass == true
                        eul = rad2deg(quat2eul(logMonteCarlo(i).loggedData.loggedData.q_bf,'XYZ'));
                        for j = 1:length(obj.evaluate_rollRange)
                            time_indices = find(ismember(floor(logMonteCarlo(i).loggedData.loggedData.time), obj.evaluate_rollRange{j}{1}));
                            for k = 1:length(time_indices)
                                if eul(time_indices(k), 1) < obj.evaluate_rollRange{j}{2}(1) || eul(time_indices(k), 1) > obj.evaluate_rollRange{j}{2}(2)
                                    logMonteCarlo(i).pass = false;
                                    fprintf('Index %d: Roll condition from second %d to %d, failed (found roll = %.2f), setting pass to false.\n', i,...
                                        obj.evaluate_rollRange{j}{1}(1), obj.evaluate_rollRange{j}{1}(end), eul(time_indices(k), 1));
                                    break;
                                end
                            end
                        end
                    end
                end
            end

            % Check if pitch is within bounds
            if obj.evaluate_checkPitch
                for i = 1:numSims
                    if logMonteCarlo(i).pass == true
                        eul = rad2deg(quat2eul(logMonteCarlo(i).loggedData.loggedData.q_bf,'XYZ'));
                        for j = 1:length(obj.evaluate_pitchRange)
                            time_indices = find(ismember(floor(logMonteCarlo(i).loggedData.loggedData.time), obj.evaluate_pitchRange{j}{1}));
                            for k = 1:length(time_indices)
                                if eul(time_indices(k), 2) < obj.evaluate_pitchRange{j}{2}(1) || eul(time_indices(k), 2) > obj.evaluate_pitchRange{j}{2}(2)
                                    logMonteCarlo(i).pass = false;
                                    fprintf('Index %d: Pitch condition from second %d to %d, failed (found pitch = %.2f), setting pass to false.\n', i,...
                                        obj.evaluate_pitchRange{j}{1}(1), obj.evaluate_pitchRange{j}{1}(end), eul(time_indices(k), 2));
                                    break;
                                end
                            end
                        end
                    end
                end
            end 

            % Check if yaw is within bounds
            if obj.evaluate_checkYaw
                for i = 1:numSims
                    if logMonteCarlo(i).pass == true
                        eul = rad2deg(quat2eul(logMonteCarlo(i).loggedData.loggedData.q_bf,'XYZ'));
                        for j = 1:length(obj.evaluate_yawRange)
                            time_indices = find(ismember(floor(logMonteCarlo(i).loggedData.loggedData.time), obj.evaluate_yawRange{j}{1}));
                            for k = 1:length(time_indices)
                                if eul(time_indices(k), 3) < obj.evaluate_yawRange{j}{2}(1) || eul(time_indices(k), 3) > obj.evaluate_yawRange{j}{2}(2)
                                    logMonteCarlo(i).pass = false;
                                    fprintf('Index %d: Yaw condition from second %d to %d, failed (found yaw = %.2f), setting pass to false.\n', i,...
                                        obj.evaluate_yawRange{j}{1}(1), obj.evaluate_yawRange{j}{1}(end), eul(time_indices(k), 3));
                                    break;
                                end
                            end
                        end
                    end
                end
            end  

            % Check if AoA is within bounds
            if obj.evaluate_checkAoA
                for i = 1:numSims
                    if logMonteCarlo(i).pass == true
                        aoa = rad2deg(logMonteCarlo(i).loggedData.loggedData.angleOfAttack);
                        for j = 1:length(obj.evaluate_AoARange)
                            time_indices = find(ismember(floor(logMonteCarlo(i).loggedData.loggedData.time), obj.evaluate_AoARange{j}{1}));
                            for k = 1:length(time_indices)
                                if aoa(time_indices(k)) < obj.evaluate_AoARange{j}{2}(1) || aoa(time_indices(k)) > obj.evaluate_AoARange{j}{2}(2)
                                    logMonteCarlo(i).pass = false;
                                    fprintf('Index %d: AoA condition from second %d to %d, failed (found aoa = %.2f), setting pass to false.\n', i,...
                                        obj.evaluate_AoARange{j}{1}(1), obj.evaluate_AoARange{j}{1}{end}, aoa(time_indices(k)));
                                    break;
                                end
                            end
                        end
                    end
                end
            end  
            timeFor = toc;
            fprintf('Elapsed time: %.2f seconds\n', timeFor);
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

        % Split the current LHS into nParts. Returns a cell array of matrices.
        % Example: splitted = MCobj.splitLHS(10);
        function splittedLHS = splitLHS(obj, nParts, outFolder)
            % If the user didn't pass an outFolder, pick a default:
            if nargin < 3 || isempty(outFolder)
                outFolder = pwd;  % Current directory by default
            end
            
            % Create the folder if it doesn't already exist
            if ~exist(outFolder, 'dir')
                mkdir(outFolder);
            end
        
            if isempty(obj.lhsSamples)
                error('lhsSamples is empty. Generate or load LHS samples first.');
            end
            if mod(obj.numRuns, nParts) ~= 0
                error('numRuns (%d) must be divisible by nParts (%d).', obj.numRuns, nParts);
            end
        
            chunkSize = obj.numRuns / nParts;
            splittedLHS = cell(1, nParts);
        
            for i = 1:nParts
                idxStart = (i-1)*chunkSize + 1;
                idxEnd   = i*chunkSize;
                lhsChunk = obj.lhsSamples(idxStart:idxEnd, :);
        
                % Store it in the cell array
                splittedLHS{i} = lhsChunk;
        
                % Save the chunk to a file in outFolder
                filename = sprintf('splittedLHS%d.mat', i);
                fullFilePath = fullfile(outFolder, filename);
                save(fullFilePath, 'lhsChunk');
                fprintf('Saved chunk %d to %s\n', i, fullFilePath);
            end
        end
    end
        
    methods (Static)

        function plotMonteCarlo(logMonteCarlo)
            figure;
            num_MonteCarloRuns = length(logMonteCarlo);
            
            % Subplot for trajectories
            plot3(0,0,0,'o','MarkerSize',10)
            hold on
            for i = 1:num_MonteCarloRuns
                plot3(logMonteCarlo(i).loggedData.loggedData.posFlat(end,2),logMonteCarlo(i).loggedData.loggedData.posFlat(end,3), ...
                    logMonteCarlo(i).loggedData.loggedData.posFlat(end,1),'x','MarkerSize', 10)
                hold on
                plot3(logMonteCarlo(i).loggedData.loggedData.posFlat(:,2),logMonteCarlo(i).loggedData.loggedData.posFlat(:,3), ...
                    logMonteCarlo(i).loggedData.loggedData.posFlat(:,1))
                hold on
            end
            grid on
           % title('\textbf{Closed Loop Trajectories for 1000 MonteCarlo Samples}', 'Interpreter', 'latex')
            xlabel('\textbf{Flat Earth Y (m)}', 'Interpreter', 'latex')
            ylabel('\textbf{Flat Earth Z (m)}', 'Interpreter', 'latex')
            zlabel('\textbf{Flat Earth X (m)}', 'Interpreter', 'latex')

            % Subplot for Euler Angles
            figure;
            for i = 1:num_MonteCarloRuns
                eul = quat2eul(logMonteCarlo(i).loggedData.loggedData.q_bf);
                plot(logMonteCarlo(i).loggedData.loggedData.time,rad2deg(eul(:,1)))
                hold on             
            end
            %title('\textbf{Yaw Angle for 1000 Monte Carlo Samples}', 'Interpreter', 'latex')
            ylabel('\textbf{Yaw Angle (deg)}', 'Interpreter', 'latex')
            xlabel('\textbf{Time (s)}', 'Interpreter', 'latex')
            grid on
            ax = gca;
            ax.XMinorTick = 'on';
            ax.YMinorTick = 'on';
            ax.MinorGridLineStyle = ':';
            ax.XMinorGrid = 'on';
            ax.YMinorGrid = 'on';
            xlim([0,12]);

            % Subplot for AoA
            % Angle of Attack %
            % subplot(2,2,4)
            % for i = 1:num_MonteCarloRuns
            %     plot(logMonteCarlo(i).loggedData.loggedData.time,rad2deg(logMonteCarlo(i).loggedData.loggedData.angleOfAttack));
            %     hold on;
            % end
            % title('Angle of Attack Monte Carlo')
            % xlabel('Time (s)','FontSize', 12)
            % ylabel('\alpha (deg)','FontSize', 12)
            % % legend('1 deg','3 deg','6 deg', '9 deg')
            % grid on
            % hold off
            % 
            % % Enable minor tick marks
            % ax = gca; % Get current axis
            % ax.XMinorTick = 'on'; % Turn on minor ticks for x-axis
            % ax.YMinorTick = 'on'; % Turn on minor ticks for y-axis
            % 
            % % Customize the spacing of minor ticks
            % ax.MinorGridLineStyle = ':'; % Optional: dotted grid for minor ticks
            % ax.XMinorGrid = 'on'; % Turn on minor grid for x-axis
            % ax.YMinorGrid = 'on'; % Turn on minor grid for y-axis
            % xlim([0,12]);

            figure;
    
            % Preallocate yaw_all for efficiency
            yaw_all = zeros(num_MonteCarloRuns, length(logMonteCarlo(1).loggedData.loggedData.q_bf));
            
            % Extract yaw angle from quaternion logs
            for i = 1:num_MonteCarloRuns
                eul = quat2eul(logMonteCarlo(i).loggedData.loggedData.q_bf);
                yaw_all(i, :) = rad2deg(eul(:,1))'; % Store each Monte Carlo run
            end
            
            % Compute statistics
            yaw_mean = mean(yaw_all, 1);
            yaw_std = std(yaw_all, 0, 1);
            yaw_3sigma = 3 * yaw_std;
            time = linspace(0, 12, length(yaw_mean));
            
            % Mission requirements
            time_1_4 = linspace(0, 3, 100);
            req_1_4 = 1 * ones(1, 100);
            time_5_6 = linspace(5, 7, 100);
            req_5_6 = 5 * ones(1, 100);
            time_11_12 = linspace(11, 12, 50);
            req_11_12 = 2 * ones(1, 50);
            
            % Plot yaw statistics
            p1 = plot(time, yaw_mean, 'b-', 'LineWidth', 3.0); hold on;
            p2 = plot(time, yaw_mean + yaw_std, 'c--', 'LineWidth', 1.2);
            plot(time, yaw_mean - yaw_std, 'c--', 'LineWidth', 1.2);
            p3 = plot(time, yaw_mean + yaw_3sigma, 'r--', 'LineWidth', 1.2);
            plot(time, yaw_mean - yaw_3sigma, 'r--', 'LineWidth', 1.2);
            
            % % Plot mission requirement constraints
            % p4 = plot(time_1_4, req_1_4, 'k--', 'LineWidth', 1.5);
            % plot(time_5_6, req_5_6, 'k--', 'LineWidth', 1.5);
            % plot(time_11_12, req_11_12, 'k--', 'LineWidth', 1.5);

            % **Shaded Requirement Areas**
            % 1. Yaw must be under 1° from 0s to 3s
            h1 = patch([0, 3, 3, 0], [-1, -1, 1, 1], 'green', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
            
            % 2. Yaw must be above 5° from 5s to 7s
            patch([4, 7, 7, 4], [5, 5, 8, 8], 'green', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
            
            % 3. Yaw must be under 2° during the 12th second
            patch([11.5, 12, 12, 11.5], [-2, -2, 2, 2], 'green', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
            
            hold off;
            
            % Formatting and labels
            %title('\textbf{Yaw Angle Mean, Standard Deviation, and 3$\sigma$ with Mission Requirements of 1000 MonteCarlo Samples}', 'Interpreter', 'latex');
            ylabel('\textbf{Yaw Angle (deg)}', 'Interpreter', 'latex')
            xlabel('\textbf{Time (s)}', 'Interpreter', 'latex')
            grid on;
            
            % Enhance minor grid
            ax = gca;
            ax.XMinorTick = 'on';
            ax.YMinorTick = 'on';
            ax.MinorGridLineStyle = ':';
            ax.XMinorGrid = 'on';
            ax.YMinorGrid = 'on';
            
            % Add legend
            legend([p1, p2, p3, h1], ...
                {'Mean Yaw', 'Mean ±1σ', 'Mean ±3σ', 'Mission Requirement'}, ...
                'Location', 'northeast');
        end
    end
end