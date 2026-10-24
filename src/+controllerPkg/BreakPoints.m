% Main Authors: Karsten Caillet (kcaillet3@gatech.edu), Conor Foley
% (mfoley40@gatech.edu)

classdef BreakPoints < handle
    properties (Constant)
        idx = StateIndices
    end

    properties
        nominalTrajLog analysisPkg.Log
        numBreakPoints (1,1) double {mustBeNonnegative}
        cumVelocityInterval (1,1) double {mustBeNonnegative} % time interval to add up velocity [s]
        tIgnition (1,1) double {mustBeNonnegative}

        % Breakpoint data arrays
        cumVelocity (1,:) double {mustBeNonnegative}     % 1 x numBreakPoints array
        time (1,:) double {mustBeNonnegative}             % 1 x numBreakPoints array
        state cell                    % cell array of state vectors {nStates x 1}
        controlInput cell             % cell array of control input vectors {nInputs x 1}
        actuatorState cell            % cell array of actuator state vectors {nActuators x 1}
        stateMatrix cell              % cell array of cell arrays {{A1}, {A2}, ...}
        controlMatrix cell            % cell array of cell arrays {{B1}, {B2}, ...}
        gain cell                     % cell array of cell arrays {{K1}, {K2}, ...}
    end

    methods

        function obj = BreakPoints(nominalTrajLog, numBreakPoints, cumVelocityInterval, tIgnition)
            obj.nominalTrajLog = nominalTrajLog;
            obj.numBreakPoints = numBreakPoints;
            obj.cumVelocityInterval = cumVelocityInterval;
            obj.tIgnition = tIgnition;

            obj.generateBreakPoints();
        end

        function generateBreakPoints(obj)
            timeNominal = obj.nominalTrajLog.time;
			
            states = obj.nominalTrajLog.states;
            stateDots = obj.nominalTrajLog.stateDots;
            controlInputs = obj.nominalTrajLog.controlInputs;
            actuatorStates = obj.nominalTrajLog.actuatorStates;

            % Stop at apogee
            [~, timeNominal, states, controlInputs, actuatorStates, stateDots] = obj.cropToApogee(timeNominal, states, controlInputs, actuatorStates, stateDots);

            % Reduce time and state to every cumVelocityInterval
            % [timeNominal, states, controlInputs, actuatorStates] = obj.reduceToInterval(timeNominal, states, controlInputs, actuatorStates);

            % Get cumulated velocity for each time (magnitude at each time step)
            vx = states(obj.idx.vx, :);
            vy = states(obj.idx.vy, :);
            vz = states(obj.idx.vz, :);
            vel = sqrt(vx.^2 + vy.^2 + vz.^2);

            cumVelocityFull = cumtrapz(timeNominal,vel);

            % Filter out breakpoints from first second of flight (no control authority)
            tIgnore = obj.tIgnition; % seconds
            validIdx = find(timeNominal > tIgnore);
            cumValid = cumVelocityFull(validIdx);

            cumTargets = linspace(cumValid(1), cumValid(end), obj.numBreakPoints);

            localIdx = interp1(cumValid, 1:numel(cumValid), cumTargets, 'nearest');
            indices = unique(validIdx(localIdx));
            obj.numBreakPoints = numel(indices);

            fprintf('BreakPoints: %d breakpoints, evenly spaced in cumulative value, t=%.2fs to t=%.2fs.\n', ...
                    numel(indices), timeNominal(indices(1)), timeNominal(indices(end)));

            % Preallocate arrays
            obj.cumVelocity = zeros(1, obj.numBreakPoints);
            obj.time = zeros(1, obj.numBreakPoints);
            obj.state = cell(1, obj.numBreakPoints);
            obj.controlInput = cell(1, obj.numBreakPoints);
            obj.actuatorState = cell(1, obj.numBreakPoints);
            obj.stateMatrix = cell(1, obj.numBreakPoints);
            obj.controlMatrix = cell(1, obj.numBreakPoints);
            obj.gain = cell(1, obj.numBreakPoints);
			%obj.refState = cell(1, obj.numBreakPoints);

            % Populate each breakpoint
            for ii = 1:obj.numBreakPoints
                idx = indices(ii);
                obj.cumVelocity(ii) = cumVelocityFull(idx);
                obj.time(ii) = timeNominal(idx);
                obj.state{ii} = states(:, idx);
                obj.controlInput{ii} = controlInputs(:, idx);
                obj.actuatorState{ii} = actuatorStates(:, idx);
                obj.stateMatrix{ii} = {};
                obj.controlMatrix{ii} = {};
                obj.gain{ii} = {};
				%obj.refState{ii} = {};
            end
        end

        function [interpolatedGains, interpolatedControlReference] = interpolateGain(obj, currentCumVelocity)
            arguments
                obj
                currentCumVelocity (1,1) double {mustBeNonnegative}
            end
            % Interpolate gain(s) and reference control based on current cumulative velocity
            %
            % Inputs:
            %   currentCumVelocity - Current cumulative velocity value
            %
            % Outputs:
            %   interpolatedGains          - Cell array of interpolated gain matrices
            %   interpolatedControlReference - Interpolated reference control input

            if isnan(currentCumVelocity)
                error("NaN currentCumVelocity")
            end
            
            % Handle edge cases
            if currentCumVelocity <= obj.cumVelocity(1)
                % Below first breakpoint - return first values
                interpolatedGains = obj.gain{1};
                interpolatedControlReference = obj.controlInput{1};
                return;
            elseif currentCumVelocity >= obj.cumVelocity(end)
                % Above last breakpoint - return last values
                interpolatedGains = obj.gain{end};
                interpolatedControlReference = obj.controlInput{end};
                return;
            end

            % Find bracketing breakpoints
            % Find largest index where cumVel <= currentCumVelocity
            lowerIdx = find(obj.cumVelocity <= currentCumVelocity, 1, 'last');
            upperIdx = lowerIdx + 1;

            % Get bracketing values
            cumVel_lower = obj.cumVelocity(lowerIdx);
            cumVel_upper = obj.cumVelocity(upperIdx);

            % Compute interpolation weight (0 at lower, 1 at upper)
            lambda = (currentCumVelocity - cumVel_lower) / (cumVel_upper - cumVel_lower);

            % Get gain cell arrays and control inputs from both breakpoints
            gains_lower = obj.gain{lowerIdx};
            gains_upper = obj.gain{upperIdx};

            controlInput_lower = obj.controlInput{lowerIdx};
            controlInput_upper = obj.controlInput{upperIdx};

            % Number of gains (should be same for all breakpoints)
            n_gains = length(gains_lower);

            % Preallocate output cell array
            interpolatedGains = cell(1, n_gains);

            % Interpolate each gain matrix
            for j = 1:n_gains
                interpolatedGains{j} = (1 - lambda) * gains_lower{j} + ...
                                        lambda * gains_upper{j};
            end

            % Interpolate reference control input
            interpolatedControlReference = (1 - lambda) * controlInput_lower + ...
                                            lambda * controlInput_upper;
        end

        function [time, states, controlInputs, actuatorStates] = reduceToInterval(obj, time, states, controlInputs, actuatorStates)
            arguments
                obj
                time (1,:) double
                states (:,:) double
                controlInputs (:,:) double
                actuatorStates (:,:) double
            end
            % Downsample to exact cumVelocityInterval using interp1
            if obj.cumVelocityInterval <= 0
                return;
            end

            % Create new time vector at exact intervals
            timeNew = time(1):obj.cumVelocityInterval:time(end);

            % Interpolate states, control inputs, and actuator states
            statesNew = interp1(time, states', timeNew, 'linear')';
            controlInputsNew = interp1(time, controlInputs', timeNew, 'linear')';
            actuatorStatesNew = interp1(time, actuatorStates', timeNew, 'linear')';

            fprintf('reduceToInterval: Resampled from %d to %d points\n', ...
                    length(time), length(timeNew));

            time = timeNew;
            states = statesNew;
            controlInputs = controlInputsNew;
            actuatorStates = actuatorStatesNew;
        end
    end

    methods (Access = private)

        function [timeOfApogee, time, states, controlInputs, actuatorStates, stateDots] = cropToApogee(obj, time, states, controlInputs, actuatorStates, stateDots)
            % Detect apogee and crop trajectory data to before apogee

            % Extract altitude and compute derivatives
            tStart = find(time>=obj.tIgnition,1);
            z = states(obj.idx.z, tStart:end); % Ignore first 1s
            [~, apogee_local] = max(z);
            apogee_idx   = apogee_local + tStart - 1;
            timeOfApogee = time(apogee_idx);

            mask = time < timeOfApogee;
            time = time(mask);
            states = states(:, mask);
            controlInputs = controlInputs(:, mask);
            actuatorStates = actuatorStates(:, mask);
            stateDots = stateDots(:, mask);
            fprintf("BreakPoints: Apogee detected at t=%.2f s - cropping nominal flight.\n", timeOfApogee);
        
        end    
    end
end

