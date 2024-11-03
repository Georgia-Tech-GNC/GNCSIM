classdef TrimBatch < handle
    % TrimBatch Class for Managing and Generating Trim Points
    % This class handles variations of a base state (e.g., rocket states) and generates
    % all possible combinations of these variations as trim points for simulation or analysis.

    properties (Constant)
        % State Names
        % A constant cell array containing the names of all possible states.
        state_names = {"u", "v", "w", "p", "q", "r", ...
                      "q0", "q1", "q2", "q3", "Roll", "Yaw", "Thrust"};
        
        % State Indices
        % A constant array representing the indices corresponding to each state name.
        state_index_con = 1:length(TrimBatch.state_names);
    end

    properties (SetAccess = private)
        % Variations
        % An array of structs, each representing a specific variation (e.g., speed, turn).
        % Each struct contains:
        %   - name: Name of the variation.
        %   - varables: Array of variable names involved in the variation.
        %   - variation_data: Matrix containing data for each variable across different states.
        variations = struct('name', {}, 'varables', {}, 'variation_data', {});
        
        % Base State
        % A row vector representing the default state of the rocket for this batch.
        base_state
        
        % Time
        % A double value indicating the time instance when this batch is taken.
        time
        
        % Batch ID
        % An integer serving as a unique identifier for this batch.
        batch_id

        % Number of Trim Points
        % An integer representing the total number of trim points generated based on variations.
        num_trim_points

        % State Dictionary
        % A containers.Map object mapping state names (as strings) to their corresponding indices.
        state_dict

        % Trim Points
        % An array of structs, each representing a generated trim point.
        % Each struct contains:
        %   - name: Name identifier for the trim point.
        %   - id: Unique identifier combining batch ID and point index.
        %   - states: Row vector representing the state configuration of the trim point.
        trim_points
    end

    methods
        function obj = TrimBatch(base_state, time, id)
            % Constructor for TrimBatch Class
            % Initializes the TrimBatch object with a base state, time, and unique identifier.
            %
            % Inputs:
            %   base_state - Default state of the rocket (vector).
            %   time       - Time instance for this batch (double).
            %   id         - Unique identifier for this batch (integer).

            % Convert state names to character arrays for compatibility with containers.Map
            state_keys = cellfun(@char, TrimBatch.state_names, 'UniformOutput', false);
            state_values = num2cell(TrimBatch.state_index_con);
            
            % Create a dictionary (Map) with state names as keys and their indices as values
            obj.state_dict = containers.Map(state_keys, state_values);       

            % Ensure base_state is a row vector
            obj.base_state = base_state(:).';  % Force to be a row vector
            obj.time = time;
            obj.batch_id = id;
        end

        function add_variation(obj, name, varables, variation_data)
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
            variation.variation_data = variation_data;    % Data associated with each variable
            
            % Append the new variation struct to the variations array
            obj.variations(end + 1) = variation;
            
            % Recalculate the total number of trim points based on new variation
            obj.count_trim_points();
        end

        function count_trim_points(obj)
            % Counts the Total Number of Trim Points
            % This method calculates the total number of trim points by taking the
            % Cartesian product of the number of states in each variation.
            
            num_trim_points_counter = 1;
            
            % Iterate through each variation to calculate the total combinations
            for ii = 1:length(obj.variations)
                variation = obj.variations(ii);
                [num_var, ~] = size(variation.variation_data);
                num_trim_points_counter = num_trim_points_counter * num_var;
            end
            
            % Store the calculated number of trim points
            obj.num_trim_points = num_trim_points_counter;
        end

        function generate_trim_batch(obj)
            % Generates a Batch of Trim Points by Creating Combinations of Each Variation's States
            % This method generates all possible combinations of variations and stores them as trim points.
            
            % Ensure that there is at least one variation added
            if isempty(obj.variations)
                error('No variations have been added. Please add at least one variation before generating trim points.');
            end
            
            % Calculate the total number of combinations (already stored in obj.num_trim_points)
            total_points = obj.num_trim_points;
    
            % Initialize the array of trim points with a template structure
            trim_point_template = struct('time', [], 'id', [], 'name', [], 'states', obj.base_state);
            obj.trim_points = repmat(trim_point_template, total_points, 1);
    
            % Create the Cartesian product of all variation states
            combination_indices = obj.generate_combinations();
    
            % Generate each trim point based on the combination indices
            for point_idx = 1:total_points
                % Initialize current_state as a copy of the base state
                current_state = obj.base_state;
                
                % Initialize trim_name as an empty string to build the name
                trim_name = "";
                
                % Get the combination indices for the current trim point
                combo_index = combination_indices(point_idx, :);
                
                % Apply each variation to the current state
                for var_idx = 1:length(obj.variations)
                    variation = obj.variations(var_idx);
    
                    % Get the row index from variation_data for this variation
                    row_idx = combo_index(var_idx);
    
                    % Append to trim_name with variation name and state index
                    trim_name = trim_name + sprintf('%s%d ', variation.name, combo_index(var_idx));
                    
                    % Update current_state based on variation variables and data
                    for col_idx = 1:length(variation.varables)
                        % Retrieve the index of the current variable in the state vector
                        state_index = obj.state_dict(variation.varables{col_idx});
                        
                        % Update the state value with the corresponding variation data
                        current_state(state_index) = variation.variation_data(row_idx, col_idx);
                    end
                end
    
                % Assign the batch time to the trim point
                obj.trim_points(point_idx).time = obj.time;

                % Assign the generated name to the current trim point
                obj.trim_points(point_idx).name = trim_name;
                
                % Create a unique ID for the trim point by combining batch ID and point index
                obj.trim_points(point_idx).id = sprintf('%d.%d', obj.batch_id, point_idx);
                
                % Store the updated state in the trim point
                obj.trim_points(point_idx).states = current_state;
                
            end
        end
    end

    methods (Access = private)
        function combinations = generate_combinations(obj)
            % Generates the Cartesian Product of Row Indices for Each Variation's Data Matrix
            % Each row in 'combinations' represents one unique trim point configuration.
            %
            % Output:
            %   combinations - A matrix where each row contains indices corresponding to each variation's data.
            
            num_variations = length(obj.variations);  % Total number of variations
            variation_sizes = zeros(1, num_variations);  % Array to store the number of states per variation
            
            % Get the number of rows (states) in each variation's data
            for i = 1:num_variations
                variation_sizes(i) = size(obj.variations(i).variation_data, 1);
            end
    
            % Generate the Cartesian product of all indices
            if num_variations == 1
                % If there's only one variation, create a column vector of indices
                combinations = (1:variation_sizes(1))';
            else
                % For multiple variations, use ndgrid to generate all possible combinations
                grid = cell(1, num_variations);  % Initialize a cell array to hold grid outputs
                
                % Create a cell array of ranges for each variation
                ranges = arrayfun(@(x) 1:x, variation_sizes, 'UniformOutput', false);
                
                % Generate grids for each variation's indices
                [grid{:}] = ndgrid(ranges{:});
                
                % Concatenate the grids and reshape to create combinations
                combinations = reshape(cat(num_variations + 1, grid{:}), [], num_variations);
            end
        end
    end
end
