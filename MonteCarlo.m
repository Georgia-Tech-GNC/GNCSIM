% Main Authors: Karsten Caillet 
classdef MonteCarlo < handle

    properties (Access = private)
        % Monte Carlo Simulation Hyperparameters and variables
        runGaussianLHS; % boolean (LHS = Latin Hypercube Sampling)
        runUniformLHS; % boolean
        runCustomLHS;
        numRuns; % integer
        odeSolverMaxTime; 
        lhsSamples;

        % Monte Carlo Pass/Fail parameters
        evaluate_checkTermination = false;
        evaluate_checkRoll = false;
        evaluate_checkPitch = false;
        evaluate_checkYaw = false;
        evaluate_checkAoA = false;
        evaluate_rollRange; % {{yawTimeRange, boundary}, {yawTimeRange, boundary}}
        evaluate_pitchRange;
        evaluate_yawRange; 
        evaluate_AoARange;

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

        function obj = set_lhsSamples(obj, value)
            validateattributes(value, {'numeric'}, {'2d', 'nonempty'}, 'set_lhsSamples', 'lhsSamples');
            obj.lhsSamples = value;
        end

        function obj = set_odeSolverMaxTime(obj, value)
            validateattributes(value, {'numeric'}, {'nonnegative', 'scalar'}, mfilename, 'odeSolverMaxTime');
            obj.odeSolverMaxTime = value;
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
            FlightObj.set_odeSolverMaxTime(obj.odeSolverMaxTime);

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

                % Temporary solution to incorporate Wind using existing
                % discrete wind parameters in the xslx table. Future improvement:
                % changing Environment class to accept continuous
                % parameters, which can be modified through LHSSampling
                % (more realistic)
                indexLHS = lhsdesign(obj.numRuns, 1);  
                indexSamples = ceil(indexLHS * 4);

                % Append the discrete parameter as an additional column:
                obj.lhsSamples = [obj.lhsSamples, indexSamples];

            elseif obj.runCustomLHS
                if isempty(obj.lhsSamples)
                    error("Please set lhsSamples with a correct Latin Hypercube Sample matrix.")
                end
            else
                error('Either runGaussianLHS, runUniformLHS or runCustomLHS must be set to true.')
            end
            
            % Preallocate struct array
            logMonteCarlo(obj.numRuns) = struct('loggedData', [], 'pass', true);

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
                EnvironmentObjIter = EnvironmentObj;
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
                thrustCurveCopy(:,2) = objCopy.lhsSamples(i,12:end-1); 
                RocketObjIter.set_thrustCurve(thrustCurveCopy);

                % Assign random variables to EnvironmentObjIter
                EnvironmentObjIter.set_index(objCopy.lhsSamples(i,end))
                fprintf('Wind Index: %d', objCopy.lhsSamples(i,end));
                
                % Run the flight simulation
                log = FlightObjCopy.runFlight(RocketObjIter, EnvironmentObjIter, AerodynamicsObj, TrimPointObj);
            
                % Store the results
                logMonteCarlo(i).loggedData = log;
            end

            timeFor = toc;
            fprintf('Elapsed time: %.2f seconds for %d iterations\n', timeFor, obj.numRuns);
        end

        function logMonteCarlo = evaluateMonteCarlo(obj, logMonteCarlo)
            tic;
            numSims = length(logMonteCarlo);

            % Check if ode solver got terminated for excessive solving time
            if obj.evaluate_checkTermination
                for i = 1:numSims
                    if logMonteCarlo(i).loggedData.loggedData.termination_flag == true
                        logMonteCarlo(i).pass = false;
                        fprintf('Index %d: termination_flag is true, setting pass to false.\n', i);
                    end
                end
            end

            if obj.evaluate_checkRoll
                for i = 1:numSims
                    if logMonteCarlo(i).pass == true
                        eul = rad2deg(quat2eul(logMonteCarlo(i).loggedData.loggedData.q_bf));
                        for j = 1:length(obj.evaluate_rollRange)
                            time_indices = find(floor(logMonteCarlo(i).loggedData.loggedData.time), obj.evaluate_rollRange{j}{1});
                            for k = 1:length(time_indices)
                                if eul(time_indices(k), 1) < obj.evaluate_rollRange{j}{2}(1) || eul(time_indices(k), 1) > obj.evaluate_rollRange{j}{2}(2)
                                    logMonteCarlo(i).pass = false;
                                    fprintf('Index %d: Yaw condition at %s failed (found eul < 5 deg), setting pass to false.\n', i);

                            


            for i = 1:length(logMonteCarlo)
                % Check if ode solver got terminated for excessive solving time
                if logMonteCarlo(i).loggedData.loggedData.termination_flag == true
                    logMonteCarlo(i).pass = false;
                    fprintf('Index %d: termination_flag is true, setting pass to false.\n', i);
                else
                    % Check if yaw gets above 5 deg at 5th second
                    indices_time_5s = find(floor(logMonteCarlo(i).loggedData.loggedData.time) == 5);
                    eul = rad2deg(quat2eul(logMonteCarlo(i).loggedData.loggedData.q_bf));
                    yaw_5s_condition = true;
                    index_5s = 1;
                    while yaw_5s_condition && index_5s <= length(indices_time_5s)
                        if eul(indices_time_5s(index_5s), 1) < 5
                            disp(eul(indices_time_5s(index_5s)))
                            yaw_5s_condition = false;
                        end
                        index_5s = index_5s + 1;
                    end 
                    if yaw_5s_condition == false
                        logMonteCarlo(i).pass = false;
                        fprintf('Index %d: Yaw condition at 5s failed (found eul < 5 deg), setting pass to false.\n', i);
                    else 
                        % Check if yaw is within +/- 2 deg around 0 at 12th second
                        indices_time_12s = find(floor(logMonteCarlo(i).loggedData.loggedData.time) == 12);
                        yaw_12s_condition = true;
                        index_12s = 1;
                        while yaw_12s_condition && index_12s <= length(indices_time_12s)
                            if eul(indices_time_12s(index_12s), 1) < -2 || eul(indices_time_12s(index_12s), 1) > 2
                                yaw_12s_condition = false;
                            end
                            index_12s = index_12s + 1;
                        end 
                        if yaw_12s_condition == false
                            logMonteCarlo(i).pass = false;
                            fprintf('Index %d: Yaw condition at 12s failed (eul not in [-2,2]), setting pass to false.\n', i);
                        else
                            % Check if roll is within +/- 2 deg around 0 & AoA is not less than 10 deg for the whole flight
                            index_all = 1;
                            roll_all_condition = true;
                            AoA_all_condition = true;
                            while roll_all_condition && AoA_all_condition && index_all <= length(eul)
                                if eul(index_all, 3) < -2 || eul(index_all, 3) > 2
                                    roll_all_condition = false; 
                                end
                                if rad2deg(logMonteCarlo(i).loggedData.loggedData.angleOfAttack(index_all)) > 10
                                    AoA_all_condition = false;
                                end
                                index_all = index_all + 1;
                            end
                            if roll_all_condition == false 
                                logMonteCarlo(i).pass = false;
                                fprintf('Index %d: Roll condition failed, setting pass to false.\n', i);
                            elseif AoA_all_condition == false
                                disp(rad2deg(logMonteCarlo(i).loggedData.loggedData.angleOfAttack(index_all)))
                                logMonteCarlo(i).pass = false;
                                fprintf('Index %d: AoA condition failed, setting pass to false.\n', i);
                            else
                                logMonteCarlo(i).pass = true;
                                fprintf('Index %d: All conditions passed, setting pass to true.\n', i);
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
    end
        
    methods (Static)

        function plotMonteCarlo(logMonteCarlo)
            figure;
            num_MonteCarloRuns = length(logMonteCarlo);
            
            % Subplot for trajectories
            subplot(2,2,1)
            plot3(0,0,0,'o','MarkerSize',10)
            hold on
            for i = 1:num_MonteCarloRuns
                plot3(logMonteCarlo(i).loggedData.loggedData.posFlat(end,2),logMonteCarlo(i).loggedData.loggedData.posFlat(end,3), ...
                    logMonteCarlo(i).loggedData.loggedData.posFlat(end,1),'x','MarkerSize',10)
                hold on
                plot3(logMonteCarlo(i).loggedData.loggedData.posFlat(:,2),logMonteCarlo(i).loggedData.loggedData.posFlat(:,3), ...
                    logMonteCarlo(i).loggedData.loggedData.posFlat(:,1))
                hold on
            end
            grid on
            title('Closed Loop Trajectories')
            xlabel('Flat Earth Y (m)')
            ylabel('Flat Earth Z (m)')
            zlabel('Flat Earth X (m)')

            % Subplot for Euler Angles
            subplot(2,2,2)
            for i = 1:num_MonteCarloRuns
                eul = quat2eul(logMonteCarlo(i).loggedData.loggedData.q_bf);
                plot(logMonteCarlo(i).loggedData.loggedData.time,rad2deg(eul(:,1)))
                hold on
            end
            title('Yaw Angle Monte Carlo')
            ylabel('Angle (deg)', 'FontSize', 12)
            xlabel('Time (s)', 'FontSize', 12)
            grid on
            ax = gca;
            ax.XMinorTick = 'on';
            ax.YMinorTick = 'on';
            ax.MinorGridLineStyle = ':';
            ax.XMinorGrid = 'on';
            ax.YMinorGrid = 'on';
        end
    end
end


       
