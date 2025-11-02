classdef TrimBatch < handle
    % TrimBatch Class for Managing and Generating Trim Points
    % This class handles variations of a base state (e.g., rocket states) and generates
    % all possible combinations of these variations as trim points for simulation or analysis.

    properties (Constant)
        % State Names
        % A constant cell array containing the names of all possible states.
        stateNames = {"u", "v", "w", "p", "q", "r", ...
                      "q0", "q1", "q2", "q3", "Roll", "Yaw", "Thrust"};

        % State Indices
        % A constant array representing the indices corresponding to each state name.
        stateIndexCon = 1:length(TrimBatch.stateNames);
    end

    properties (SetAccess = private)
        % Variations
        % An array of structs, each representing a specific variation (e.g., speed, turn).
        % Each struct contains:
        %   - name: Name of the variation.
        %   - varables: Array of variable names involved in the variation.
        %   - variationData: Matrix containing data for each variable across different states.
        variations = struct('name', {}, 'varables', {}, 'variationData', {});

        % Base State
        % A row vector representing the default state of the rocket for this batch.
        baseState

        % Time
        % A double value indicating the time instance when this batch is taken.
        time

        % Batch ID
        % An integer serving as a unique identifier for this batch.
        batchId

        % Number of Trim Points
        % An integer representing the total number of trim points generated based on variations.
        numTrimPoints

        % State Dictionary
        % A containers.Map object mapping state names (as strings) to their corresponding indices.
        stateDict

        % Trim Points
        % An array of structs, each representing a generated trim point.
        % Each struct contains:
        %   - name: Name identifier for the trim point.
        %   - id: Unique identifier combining batch ID and point index.
        %   - states: Row vector representing the state configuration of the trim point.
        trimPoints
    end

    methods
        function obj = TrimBatch(baseState, time, id)
            % Constructor for TrimBatch Class
            % Initializes the TrimBatch object with a base state, time, and unique identifier.
            %
            % Inputs:
            %   baseState - Default state of the rocket (vector).
            %   time       - Time instance for this batch (double).
            %   id         - Unique identifier for this batch (integer).

            % Convert state names to character arrays for compatibility with containers.Map
            stateKeys = cellfun(@char, TrimBatch.stateNames, 'UniformOutput', false);
            stateValues = num2cell(TrimBatch.stateIndexCon);

            % Create a dictionary (Map) with state names as keys and their indices as values
            obj.stateDict = containers.Map(stateKeys, stateValues);

            % Ensure baseState is a row vector
            obj.baseState = baseState(:).';  % Force to be a row vector
            obj.time = time;
            obj.batchId = id;
        end

        function addVariation(obj, name, varables, variationData)
            % Adds a New Variation to the TrimBatch Instance
            % This method appends a new variation struct to the variations array.
            %
            % Inputs:
            %   name           - Name of the variation (e.g., 'speed', 'turn').
            %   varables       - Array of strings representing variable names involved in this variation 
            %                    (e.g., ["q0", "q1", "q2", "q3"] or ["u"]).
            %   variation_data - Matrix where each row represents a different state, and each column 
            %                    corresponds to a variable in 'varables'.
            %
            % Example:
            %   name = 'turn';
            %   varables = ["q0", "q1", "q2", "q3"];
            %   variation_data = [0.99, 0, 0, 0.08; 
            %                     0.98, 0, 0, 0.16];
            
            % Define a struct to store variation information
            variation.name = name;                        % Name of the variation
            variation.varables = varables;                % Array of variable names as strings
            variation.variationData = variationData;    % Data associated with each variable

            % Append the new variation struct to the variations array
            obj.variations(end + 1) = variation;

            % Recalculate the total number of trim points based on new variation
            obj.countTrimPoints();
        end

        function countTrimPoints(obj)
            % Counts the Total Number of Trim Points
            % This method calculates the total number of trim points by taking the
            % Cartesian product of the number of states in each variation.

            numTrimPointsCounter = 1;

            % Iterate through each variation to calculate the total combinations
            for ii = 1:length(obj.variations)
                variation = obj.variations(ii);
                [numVar, ~] = size(variation.variationData);
                numTrimPointsCounter = numTrimPointsCounter * numVar;
            end

            % Store the calculated number of trim points
            obj.numTrimPoints = numTrimPointsCounter;
        end

        function generateTrimBatch(obj)
            % Generates a Batch of Trim Points by Creating Combinations of Each Variation's States
            % This method generates all possible combinations of variations and stores them as trim points.
            
            % Ensure that there is at least one variation added
            if isempty(obj.variations)
                error('No variations have been added. Please add at least one variation before generating trim points.');
            end

            % Calculate the total number of combinations (already stored in obj.numTrimPoints)
            totalPoints = obj.numTrimPoints;

            % Initialize the array of trim points with a template structure
            trimPointTemplate = struct('time', [], 'id', [], 'name', [], 'states', obj.baseState);
            obj.trimPoints = repmat(trimPointTemplate, totalPoints, 1);

            % Create the Cartesian product of all variation states
            combinationIndices = obj.generateCombinations();

            % Generate each trim point based on the combination indices
            for pointIdx = 1:totalPoints
                % Initialize currentState as a copy of the base state
                currentState = obj.baseState;

                % Initialize trimName as an empty string to build the name
                trimName = "";

                % Get the combination indices for the current trim point
                comboIndex = combinationIndices(pointIdx, :);

                % Apply each variation to the current state
                for varIdx = 1:length(obj.variations)
                    variation = obj.variations(varIdx);

                    % Get the row index from variationData for this variation
                    rowIdx = comboIndex(varIdx);

                    % Append to trimName with variation name and state index
                    trimName = trimName + sprintf('%s%d ', variation.name, comboIndex(varIdx));

                    % Update currentState based on variation variables and data
                    for colIdx = 1:length(variation.varables)
                        % Retrieve the index of the current variable in the state vector
                        stateIndex = obj.stateDict(variation.varables{colIdx});

                        % Update the state value with the corresponding variation data
                        currentState(stateIndex) = variation.variationData(rowIdx, colIdx);
                    end
                end

                % Assign the batch time to the trim point
                obj.trimPoints(pointIdx).time = obj.time;

                % Assign the generated name to the current trim point
                obj.trimPoints(pointIdx).name = trimName;

                % Create a unique ID for the trim point by combining batch ID and point index
                obj.trimPoints(pointIdx).id = sprintf('%d.%d', obj.batchId, pointIdx);

                % Store the updated state in the trim point
                obj.trimPoints(pointIdx).states = currentState;

            end
        end
    end

    methods (Access = private)
        function combinations = generateCombinations(obj)
            % Generates the Cartesian Product of Row Indices for Each Variation's Data Matrix
            % Each row in 'combinations' represents one unique trim point configuration.
            %
            % Output:
            %   combinations - A matrix where each row contains indices corresponding to each variation's data.

            numVariations = length(obj.variations);  % Total number of variations
            variationSizes = zeros(1, numVariations);  % Array to store the number of states per variation

            % Get the number of rows (states) in each variation's data
            for i = 1:numVariations
                variationSizes(i) = size(obj.variations(i).variationData, 1);
            end

            % Generate the Cartesian product of all indices
            if numVariations == 1
                % If there's only one variation, create a column vector of indices
                combinations = (1:variationSizes(1))';
            else
                % For multiple variations, use ndgrid to generate all possible combinations
                grid = cell(1, numVariations);  % Initialize a cell array to hold grid outputs

                % Create a cell array of ranges for each variation
                ranges = arrayfun(@(x) 1:x, variationSizes, 'UniformOutput', false);

                % Generate grids for each variation's indices
                [grid{:}] = ndgrid(ranges{:});

                % Concatenate the grids and reshape to create combinations
                combinations = reshape(cat(numVariations + 1, grid{:}), [], numVariations);
            end
        end
    end
end
