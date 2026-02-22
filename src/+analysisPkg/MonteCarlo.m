classdef MonteCarlo < handle
    properties
        numRuns (1,1) double 
        lhsSamples (:,:) double
        uncertainty analysisPkg.Uncertainty

        flight flightPkg.Flight
        rocketDynamics rocketPkg.RocketDynamics
        environment flightPkg.Environment
        logTemplate analysisPkg.Log
    end

    methods
        function obj = MonteCarlo(uncertainty, flight, rocketDynamics, environment, logTemplate, numRuns)
            arguments
                uncertainty 
                flight 
                rocketDynamics 
                environment
                logTemplate 
                numRuns 
            end
            obj.uncertainty = uncertainty;
            obj.flight = flight;
            obj.rocketDynamics = rocketDynamics;
            obj.environment = environment;
            obj.logTemplate = logTemplate;
            obj.numRuns = numRuns;

            % Generate LHS samples
            obj.lhsSamples = uncertainty.generateLhsSamples(numRuns);
        end

        function logs = runMonteCarlo(obj)
            pool = gcp('nocreate');
            if isempty(pool)
                error('A parallel pool is required before calling runMonteCarlo. Start one with parpool(...).');
            end

            % Pre-allocate cell array for logs
            logs = cell(obj.numRuns, 1);

            % Get variable names for indexing
            numVars = length(obj.uncertainty.variables);
            varNames = cell(numVars, 1);
            for i = 1:numVars
                varNames{i} = obj.uncertainty.variables(i).name;
            end

            % Extract samples for parallel loop
            samples = obj.lhsSamples;

            % Run Monte Carlo simulations in parallel
            parfor runIdx = 1:obj.numRuns
                fprintf('Running Monte Carlo iteration %d of %d\n', runIdx, obj.numRuns);

                % Create copies for this iteration (needed for parfor)
                flightCopy = obj.flight.copy();
                rocketCopy = obj.rocketDynamics.copy();
                environmentCopy = obj.environment.copy();
                
                % Manually deep copy parachute linked list
                rocketCopy.activeParachute = analysisPkg.MonteCarlo.copyParachuteList(obj.rocketDynamics.activeParachute);

                % Apply uncertainty parameters for this run
                analysisPkg.MonteCarlo.applyUncertaintyParameters(rocketCopy, environmentCopy, flightCopy, samples(runIdx, :), varNames);

                % Update flight with modified rocket
                flightCopy.rocketDynamics = rocketCopy;

                % Copy log template
                log = obj.logTemplate.copy();

                % Run simulation
                log = flightCopy.runSim(log);
                log.trim();

                % Store log
                logs{runIdx} = log;
            end
        end
    end

    methods (Static, Access = private)
        function applyUncertaintyParameters(rocketDynamics, environment, flight, sampleValues, varNames)
            arguments
                rocketDynamics rocketPkg.RocketDynamics
                environment flightPkg.Environment
                flight flightPkg.Flight
                sampleValues (1,:) double
                varNames cell
            end

            % Store original motor parameters
            originalTotalImpulse = rocketDynamics.totalImpulse;
            originalBurnTime = rocketDynamics.burnTime;
            originalTIgnition = rocketDynamics.tIgnition;
            originalmassWet = rocketDynamics.massWet;
            originalmassDry = rocketDynamics.massDry;
            
            % Calculate nominal Isp 
            g0 = 9.81; 
            nominalMassPropellant = originalmassWet - originalmassDry;
            nominalIsp = originalTotalImpulse / (nominalMassPropellant * g0); 

            % Apply each uncertainty parameter
            for i = 1:length(varNames)
                value = sampleValues(i);

                switch varNames{i}
                    % Aerodynamic scale factors
                    case 'caScaleFactor'
                        rocketDynamics.body.scaleFactor = value;
                    case 'cmScaleFactor'
                        rocketDynamics.body.momentScaleFactor = value;
                    case 'cnScaleFactor'
                        % Apply to fins
                        [~, numFins] = size(rocketDynamics.rocketFins.fins);
                        for j = 1:numFins
                            rocketDynamics.rocketFins.fins(j).scaleFactor = value;
                        end

                    % Mass distribution
                    case 'wetMass'
                        rocketDynamics.massWet = value;
                    case 'cgXWet'
                        rocketDynamics.cgWet = value;
                    case 'cgXDry'
                        rocketDynamics.cgDry = value;
                    case 'longitudinalMoiWet'
                        rocketDynamics.moiWet(2,2) = value;
                        rocketDynamics.moiWet(3,3) = value;
                    case 'rotationalMoiWet'
                        rocketDynamics.moiWet(1,1) = value;
                    case 'longitudinalMoiDry'
                        rocketDynamics.moiDry(2,2) = value;
                        rocketDynamics.moiDry(3,3) = value;
                    case 'rotationalMoiDry'
                        rocketDynamics.moiDry(1,1) = value;

                    % Motor parameters
                    case 'totalImpulse'
                        rocketDynamics.totalImpulse = value;
                    case 'burnTime'
                        rocketDynamics.burnTime = value;
                    case 'tIgnition'
                        rocketDynamics.tIgnition = value;

                    % Rail parameters
                    case 'railLength'
                        flight.flightParams.launchRailLength = value;
                    case 'railOrientation'
                        % Apply orientation perturbation to initial quaternion
                        % This would modify the initial state

                    % Body geometry
                    case 'noseLength'
                        rocketDynamics.body.bodyParams.noseLength = value;
                    case 'bodyLength'
                        rocketDynamics.body.bodyParams.bodyLength = value;
                    case 'rocketDiameter'
                        rocketDynamics.body.bodyParams.rocketDiameter = value;

                    % Actuator parameters
                    case 'maxDeflection'
                        rocketDynamics.actuatorModel.limits.maxDeflection = value;
                    case 'maxRate'
                        rocketDynamics.actuatorModel.limits.maxRate = value;
                    case 'timeConstant'
                        rocketDynamics.actuatorModel.timeConstant = value;

                     % Wind
                    case 'U0'
                        environment.U0 = value;
                    case 'theta0'
                        environment.theta0 = value;
                    case 'I0'
                        environment.I0 = value;
                    
                    % Parachute    
                    otherwise
                        if startsWith(varNames{i}, 'parachuteCd_')
                            
                            parachuteName = varNames{i}(13:end);  
                            currentParachute = rocketDynamics.activeParachute;
                            found = false;

                            while ~isempty(currentParachute) && ~found % not most efficient but not large number of Parachutes
                                if strcmp(currentParachute.name, parachuteName)
                                    currentParachute.dragCoefficient = value;
                                    found = true;
                                else
                                    currentParachute = currentParachute.nextParachute;
                                end
                            end
                        end
                end
            end
            

            % Dependent parameters values 

            % thrustCurve
            timeScaleFactor = rocketDynamics.burnTime / originalBurnTime;
            thrustScaleFactor = (rocketDynamics.totalImpulse / originalTotalImpulse) / timeScaleFactor;
            rocketDynamics.thrustCurve(:,1) = (rocketDynamics.thrustCurve(:,1) - originalTIgnition) * timeScaleFactor + rocketDynamics.tIgnition;
            rocketDynamics.thrustCurve(:,2) = rocketDynamics.thrustCurve(:,2) * thrustScaleFactor;
            
            % Dry mass calculation 
            massPropellant = rocketDynamics.totalImpulse / (nominalIsp * g0);
            rocketDynamics.massDry = rocketDynamics.massWet - massPropellant;

        end
        
        function parachuteCopy = copyParachuteList(parachute)
            % Recursively copy parachute linked list
            if isempty(parachute)
                parachuteCopy = rocketPkg.ParachuteDynamics.empty;
                return;
            end
            
            % Create new parachute with same properties
            parachuteCopy = rocketPkg.ParachuteDynamics(...
                parachute.name, ...
                parachute.canopyDiameter, ...
                parachute.canopyMass, ...
                parachute.dragCoefficient, ...
                parachute.deployAltitude);
            
            % Copy deployment state
            parachuteCopy.isDeployed = parachute.isDeployed;
            
            % Recursively copy next parachute
            if ~isempty(parachute.nextParachute)
                parachuteCopy.nextParachute = analysisPkg.MonteCarlo.copyParachuteList(parachute.nextParachute);
            end
        end
    end
end