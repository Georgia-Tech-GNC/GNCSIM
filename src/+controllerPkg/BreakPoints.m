% Main Authors: Karsten Caillet (kcaillet3@gatech.edu)

classdef BreakPoints < handle
    properties (Constant)
        idx = StateIndices
    end

    properties
        nominalTrajLog
        numBreakPoints
        cumVelocityInterval % time interval to add up velocity [s]

        % Breakpoint data arrays
        cumVelocity      % 1 x numBreakPoints array
        time             % 1 x numBreakPoints array
        state            % cell array of state vectors {nStates x 1}
        controlInput     % cell array of control input vectors {nInputs x 1}
        actuatorState    % cell array of actuator state vectors {nActuators x 1}
        stateMatrix      % cell array of cell arrays {{A1}, {A2}, ...}
        controlMatrix    % cell array of cell arrays {{B1}, {B2}, ...}
        gain             % cell array of cell arrays {{K1}, {K2}, ...}
    end

    methods

        function obj = BreakPoints(nominalTrajLog, numBreakPoints, cumVelocityInterval)
            obj.nominalTrajLog = nominalTrajLog;
            obj.numBreakPoints = numBreakPoints;
            obj.cumVelocityInterval = cumVelocityInterval;

            obj.generateBreakPoints();
        end

        function generateBreakPoints(obj)
            timeNominal = obj.nominalTrajLog.time;
			
            states = obj.nominalTrajLog.states;
            controlInputs = obj.nominalTrajLog.controlInputs;
            actuatorStates = obj.nominalTrajLog.actuatorStates;

            % Stop at apogee
            [~, timeNominal, states, controlInputs, actuatorStates] = obj.cropToApogee(timeNominal, states, controlInputs, actuatorStates);

            % Reduce time and state to every cumVelocityInterval
            [timeNominal, states, controlInputs, actuatorStates] = obj.reduceToInterval(timeNominal, states, controlInputs, actuatorStates);

            % Get cumulated velocity for each time (magnitude at each time step)
            vx = states(obj.idx.vx, :);
            vy = states(obj.idx.vy, :);
            vz = states(obj.idx.vz, :);
            vel = sqrt(vx.^2 + vy.^2 + vz.^2);
			dt = timeNominal(2) - timeNominal(1);
            cumVelocityFull = cumsum(vel * dt);

            % Get breakpoint indices
            indices = round(linspace(1, length(cumVelocityFull), obj.numBreakPoints));

            % Filter out breakpoints from first second of flight (no control authority)
            tIgnore = 1.0; % seconds
            validMask = timeNominal(indices) > tIgnore;
            indices = indices(validMask);
            numValidBreakPoints = length(indices);

            fprintf('BreakPoints: Filtered out %d breakpoints from first %.1f seconds (no control authority).\n', ...
                    obj.numBreakPoints - numValidBreakPoints, tIgnore);
            fprintf('BreakPoints: Using %d breakpoints from t=%.2fs to t=%.2fs.\n', ...
                    numValidBreakPoints, timeNominal(indices(1)), timeNominal(indices(end)));

            % Update numBreakPoints to actual count
            obj.numBreakPoints = numValidBreakPoints;

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

        function [timeOfApogee, time, states, controlInputs, actuatorStates] = cropToApogee(obj, time, states, controlInputs, actuatorStates)
            % Detect apogee and crop trajectory data to before apogee

            % Extract altitude and compute derivatives
            time_derivative = time(100:end); % Ignore first 1s
            z = states(obj.idx.z, 100:end); % Ignore first 1s
            dz = gradient(z, time_derivative);   % Vertical velocity
            d2z = gradient(dz, time_derivative); % Vertical acceleration

            % Find points where vertical velocity is near zero
            index = find(abs(dz) < 0.1);

            % Filter for points with negative acceleration (true apogee)
            valid_idx = index(d2z(index) < 0);

            if ~isempty(valid_idx)
                % Use first valid point as apogee
                apogee_idx = valid_idx(1);
                timeOfApogee = time(apogee_idx);

                % Crop all data to before apogee
                mask = time < timeOfApogee;
                time = time(mask);
                states = states(:, mask);
                controlInputs = controlInputs(:, mask);
                actuatorStates = actuatorStates(:, mask);
                fprintf("BreakPoints: Apogee detected at t=%.2f - cropping nominal flight.\n", timeOfApogee)
            else
                timeOfApogee = 0;
                fprintf("BreakPoints: No apogee detected - using full nominal flight.")
            end
        end    
    end
end

