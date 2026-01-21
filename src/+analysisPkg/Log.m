% Main Authors: Karsten Caillet (kcaillet3@gatech.edu)

classdef Log < handle
    properties
        time                    % 1 x N
        states                  % nStates x N
        stateDots               % nStates x N
        stateEstimates          % nStates x N
        ekfCovariances
        controlInputs           % nControlInputs x N
        controlMoments          % 3 x N
        controlForces           % 3 x N
        aeroForces              % 3 x N
        aeroMoments             % 3 x N
        thrust                  % 1 x N
        actuatorStates           % nActuatorStates x N

        stateNames              % Cell array of state names
        stateDotNames
        ekfCovarianceNames
        controlInputNames       % Cell array of control input names
        controlMomentNames      % Cell array of control moment names
        controlForcesNames      % Cell array of control force names
        aeroForcesNames
        aeroMomentNames
        actuatorStateNames      % Cell array of actuator state names

        maxSteps                % Preallocated size
        currentIdx              % Current write index
    end
    
    methods
        function obj = Log(stateNames, controlInputNames, actuatorStateNames, maxSteps)
            % Initialize logger
            obj.stateNames = stateNames;
            obj.stateDotNames = stateNames;  % Same names with 'Dot' suffix implied
            obj.ekfCovarianceNames = {'Px', 'Py', 'Pz', 'Vx', 'Vy', 'Vz'};
            obj.controlInputNames = controlInputNames;
            obj.controlMomentNames = {'Mx', 'My', 'Mz'};
            obj.controlForcesNames = {'Fx', 'Fy', 'Fz'};
            obj.aeroForcesNames = {'Fx', 'Fy', 'Fz'};
            obj.aeroMomentNames = {'Mx', 'My', 'Mz'};
            obj.actuatorStateNames = actuatorStateNames;
            obj.maxSteps = maxSteps;
            obj.currentIdx = 0;

            % Preallocate arrays
            obj.time = zeros(1, obj.maxSteps);
            obj.states = zeros(length(obj.stateNames), obj.maxSteps);
            obj.stateDots = zeros(length(obj.stateNames), obj.maxSteps);
            obj.stateEstimates = zeros(length(obj.stateNames), obj.maxSteps);
            obj.ekfCovariances = zeros(length(obj.ekfCovarianceNames), obj.maxSteps);
            obj.controlInputs = zeros(length(obj.controlInputNames), obj.maxSteps);
            obj.controlMoments = zeros(length(obj.controlMomentNames), obj.maxSteps);
            obj.controlForces = zeros(3, obj.maxSteps);
            obj.aeroForces = zeros(3, obj.maxSteps);
            obj.aeroMoments = zeros(3, obj.maxSteps);
            obj.thrust = zeros(1, obj.maxSteps);
            obj.actuatorStates = zeros(length(obj.actuatorStateNames), obj.maxSteps);
        end

        function initializeTimestep(obj, t)
            % Initialize a new timestep
            obj.currentIdx = obj.currentIdx + 1;
            
            if obj.currentIdx > obj.maxSteps
                warning('Logger capacity exceeded. Expanding arrays...');
                obj.expand();
            end
            
            obj.time(obj.currentIdx) = t;
        end
        
        function add(obj, t, attributeName, value)
            % Add a vector/array to specified attribute at current timestep
            % attributeName: 'states', 'stateDots', 'stateEstimates', 'ekfCovariances',
            %                'controlInputs', 'controlMoments', 'controlForces', 'aeroForces', 'aeroMoments', 'thrust', 'ActuatorStates'
            % value: vector/array of values to add (must match dimension)

            if obj.currentIdx < 1
                error('Must call initializeTimestep(t) before adding data');
            end

            if abs(obj.time(obj.currentIdx) - t) > 1e-10
                error('Time mismatch: expected t=%.6f, got t=%.6f', ...
                      obj.time(obj.currentIdx), t);
            end

            switch lower(attributeName)
                case 'states'
                    if length(value) ~= length(obj.stateNames)
                        error('State vector size mismatch: expected %d, got %d', ...
                              length(obj.stateNames), length(value));
                    end
                    obj.states(:, obj.currentIdx) = value;

                case 'statedots'
                    if length(value) ~= length(obj.stateDotNames)
                        error('State derivative vector size mismatch: expected %d, got %d', ...
                              length(obj.stateDotNames), length(value));
                    end
                    obj.stateDots(:, obj.currentIdx) = value;

                case 'stateestimates'
                    if length(value) ~= length(obj.stateNames)
                        error('State estimate vector size mismatch: expected %d, got %d', ...
                              length(obj.stateNames), length(value));
                    end
                    obj.stateEstimates(:, obj.currentIdx) = value;

                case 'ekfcovariances'
                    if length(value) ~= length(obj.ekfCovarianceNames)
                        error('EKF covariance vector size mismatch: expected %d, got %d', ...
                              length(obj.ekfCovarianceNames), length(value));
                    end
                    obj.ekfCovariances(:, obj.currentIdx) = value;

                case 'controlinputs'
                    if length(value) ~= length(obj.controlInputNames)
                        error('Control input vector size mismatch: expected %d, got %d', ...
                              length(obj.controlInputNames), length(value));
                    end
                    obj.controlInputs(:, obj.currentIdx) = value;

                case 'controlmoments'
                    if length(value) ~= length(obj.controlMomentNames)
                        error('Control moment vector size mismatch: expected %d, got %d', ...
                              length(obj.controlMomentNames), length(value));
                    end
                    obj.controlMoments(:, obj.currentIdx) = value;

                case 'controlforces'
                    if length(value) ~= 3
                        error('Control forces vector size mismatch: expected 3, got %d', ...
                              length(value));
                    end
                    obj.controlForces(:, obj.currentIdx) = value;

                case 'aeroforces'
                    if length(value) ~= 3
                        error('Aero forces vector size mismatch: expected 3, got %d', ...
                              length(value));
                    end
                    obj.aeroForces(:, obj.currentIdx) = value;

                case 'aeromoments'
                    if length(value) ~= 3
                        error('Aero moments vector size mismatch: expected 3, got %d', ...
                              length(value));
                    end
                    obj.aeroMoments(:, obj.currentIdx) = value;

                case 'thrust'
                    if length(value) ~= 1
                        error('Thrust value size mismatch: expected 1, got %d', ...
                              length(value));
                    end
                    obj.thrust(obj.currentIdx) = value;

                case 'actuatorstates'
                    if length(value) ~= length(obj.actuatorStateNames)
                        error('Actuator state vector size mismatch: expected %d, got %d', ...
                              length(obj.actuatorStateNames), length(value));
                    end
                    obj.actuatorStates(:, obj.currentIdx) = value;

                otherwise
                    error('Unknown attribute "%s". Valid options: states, stateDots, stateEstimates, ekfCovariances, controlInputs, controlMoments, controlForces, aeroForces, aeroMoments, thrust, actuatorStates', ...
                          attributeName);
            end
        end
        
        function trim(obj)
            % Remove unused preallocated space
            obj.time = obj.time(1:obj.currentIdx);
            obj.states = obj.states(:, 1:obj.currentIdx);
            obj.stateDots = obj.stateDots(:, 1:obj.currentIdx);
            obj.stateEstimates = obj.stateEstimates(:, 1:obj.currentIdx);
            obj.ekfCovariances = obj.ekfCovariances(:, 1:obj.currentIdx);
            obj.controlInputs = obj.controlInputs(:, 1:obj.currentIdx);
            obj.controlMoments = obj.controlMoments(:, 1:obj.currentIdx);
            obj.controlForces = obj.controlForces(:, 1:obj.currentIdx);
            obj.aeroForces = obj.aeroForces(:, 1:obj.currentIdx);
            obj.aeroMoments = obj.aeroMoments(:, 1:obj.currentIdx);
            obj.thrust = obj.thrust(1:obj.currentIdx);
            obj.actuatorStates = obj.actuatorStates(:, 1:obj.currentIdx);
        end
        
        function expand(obj)
            % Double capacity
            newSize = obj.maxSteps * 2;
            extraCols = newSize - obj.maxSteps;

            obj.time = [obj.time, zeros(1, extraCols)];
            obj.states = [obj.states, zeros(size(obj.states, 1), extraCols)];
            obj.stateDots = [obj.stateDots, zeros(size(obj.stateDots, 1), extraCols)];
            obj.stateEstimates = [obj.stateEstimates, zeros(size(obj.stateEstimates, 1), extraCols)];
            obj.ekfCovariances = [obj.ekfCovariances, zeros(size(obj.ekfCovariances, 1), extraCols)];
            obj.controlInputs = [obj.controlInputs, zeros(size(obj.controlInputs, 1), extraCols)];
            obj.controlMoments = [obj.controlMoments, zeros(size(obj.controlMoments, 1), extraCols)];
            obj.controlForces = [obj.controlForces, zeros(3, extraCols)];
            obj.aeroForces = [obj.aeroForces, zeros(3, extraCols)];
            obj.aeroMoments = [obj.aeroMoments, zeros(3, extraCols)];
            obj.thrust = [obj.thrust, zeros(1, extraCols)];
            obj.actuatorStates = [obj.actuatorStates, zeros(size(obj.actuatorStates, 1), extraCols)];

            obj.maxSteps = newSize;
        end
        
        function data = getState(obj, name)
            % Get specific state by name
            idx = find(strcmp(obj.stateNames, name), 1);
            if isempty(idx)
                error('State "%s" not found', name);
            end
            data = obj.states(idx, 1:obj.currentIdx);
        end
        
        function data = getStateDot(obj, name)
            % Get specific state derivative by name
            idx = find(strcmp(obj.stateDotNames, name), 1);
            if isempty(idx)
                error('State derivative "%s" not found', name);
            end
            data = obj.stateDots(idx, 1:obj.currentIdx);
        end
        
        function data = getStateEstimate(obj, name)
            % Get specific state estimate by name
            idx = find(strcmp(obj.stateNames, name), 1);
            if isempty(idx)
                error('State "%s" not found', name);
            end
            data = obj.stateEstimates(idx, 1:obj.currentIdx);
        end
        
        function data = getControlInput(obj, name)
            % Get specific control input by name
            idx = find(strcmp(obj.controlInputNames, name), 1);
            if isempty(idx)
                error('Control input "%s" not found', name);
            end
            data = obj.controlInputs(idx, 1:obj.currentIdx);
        end
        
        function data = getControlMoment(obj, name)
            % Get specific control moment by name
            idx = find(strcmp(obj.controlMomentNames, name), 1);
            if isempty(idx)
                error('Control moment "%s" not found', name);
            end
            data = obj.controlMoments(idx, 1:obj.currentIdx);
        end

        function data = getControlForce(obj, name)
            % Get specific control force by name (Fx, Fy, Fz)
            idx = find(strcmp(obj.controlForcesNames, name), 1);
            if isempty(idx)
                error('Control force "%s" not found. Valid: Fx, Fy, Fz', name);
            end
            data = obj.controlForces(idx, 1:obj.currentIdx);
        end

        function data = getAeroForce(obj, name)
            % Get specific aero force by name (Fx, Fy, Fz)
            idx = find(strcmp(obj.aeroForcesNames, name), 1);
            if isempty(idx)
                error('Aero force "%s" not found. Valid: Fx, Fy, Fz', name);
            end
            data = obj.aeroForces(idx, 1:obj.currentIdx);
        end
        
        function data = getAeroMoment(obj, name)
            % Get specific aero moment by name (Mx, My, Mz)
            idx = find(strcmp(obj.aeroMomentNames, name), 1);
            if isempty(idx)
                error('Aero moment "%s" not found. Valid: Mx, My, Mz', name);
            end
            data = obj.aeroMoments(idx, 1:obj.currentIdx);
        end

        function data = getThrust(obj)
            % Get thrust data
            data = obj.thrust(1:obj.currentIdx);
        end

        function data = getEkfCovariance(obj, name)
            % Get specific EKF covariance by name (Px, Py, Pz, Vx, Vy, Vz)
            idx = find(strcmp(obj.ekfCovarianceNames, name), 1);
            if isempty(idx)
                error('EKF covariance "%s" not found. Valid: Px, Py, Pz, Vx, Vy, Vz', name);
            end
            data = obj.ekfCovariances(idx, 1:obj.currentIdx);
        end

        function data = getActuatorState(obj, name)
            % Get specific actuator state by name
            idx = find(strcmp(obj.actuatorStateNames, name), 1);
            if isempty(idx)
                error('Actuator state "%s" not found', name);
            end
            data = obj.actuatorStates(idx, 1:obj.currentIdx);
        end

        function saveToFile(obj, filename)
            % Save to .mat file
            data.time = obj.time(1:obj.currentIdx);
            data.states = obj.states(:, 1:obj.currentIdx);
            data.stateDots = obj.stateDots(:, 1:obj.currentIdx);
            data.stateEstimates = obj.stateEstimates(:, 1:obj.currentIdx);
            data.ekfCovariances = obj.ekfCovariances(:, 1:obj.currentIdx);
            data.controlInputs = obj.controlInputs(:, 1:obj.currentIdx);
            data.controlMoments = obj.controlMoments(:, 1:obj.currentIdx);
            data.controlForces = obj.controlForces(:, 1:obj.currentIdx);
            data.aeroForces = obj.aeroForces(:, 1:obj.currentIdx);
            data.aeroMoments = obj.aeroMoments(:, 1:obj.currentIdx);
            data.thrust = obj.thrust(1:obj.currentIdx);
            data.actuatorState = obj.actuatorStates(:, 1:obj.currentIdx);
            data.stateNames = obj.stateNames;
            data.stateDotNames = obj.stateDotNames;
            data.ekfCovarianceNames = obj.ekfCovarianceNames;
            data.controlInputNames = obj.controlInputNames;
            data.controlMomentNames = obj.controlMomentNames;
            data.controlForcesNames = obj.controlForcesNames;
            data.aeroForcesNames = obj.aeroForcesNames;
            data.aeroMomentNames = obj.aeroMomentNames;
            data.actuatorStateNames = obj.actuatorStateNames;
            save(filename, 'data');
        end
    end
end