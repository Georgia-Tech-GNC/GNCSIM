classdef TrimBatch < handle

    properties (Constant)
        % Define state names
        state_names = {"u", "v", "w", "p", "q", "r", "q0", "q1", "q2", "q3", "Roll", "Yaw", "Thrust"};
        
        % State indices for corresponding state names
        state_index_con = 1:length(TrimBatch.state_names);
    end

    properties (SetAccess = private)
        % Array of variation structs, each storing a specific type of variation (e.g., speed, turn)
        variations = struct('name', {}, 'varables', {}, 'variation_data', {});
        
        % Default state of rocket for this batch or time (vector)
        base_state
        
        % Time batch is taken (double)
        time
        
        % Unique identifier for this batch (int)
        batch_id

        num_trim_points

        state_dict

        trim_points
    end

    methods
        function obj = TrimBatch(base_state, time, id)
            % Constructor for TrimBatch class
            
            % Inputs:
            %   base_state - Default state of rocket. (state; contol)
            %   time       - Time instance for this batch (double)
            %   id         - Unique identifier for this batch (int)


            % Convert state names to character arrays if necessary
            state_keys = cellfun(@char, TrimBatch.state_names, 'UniformOutput', false);
            state_values = num2cell(TrimBatch.state_index_con);
            
            % Dictionary (Map) with state names as keys and indices as values
            obj.state_dict = containers.Map(state_keys, state_values);       

            obj.base_state = base_state(:).';  % force to be a row vector
            obj.time = time;
            obj.batch_id = id;
        end

        function add_variation(obj, name, varables, variation_data)
            % Adds a new variation to the TrimBatch instance.
            
            % Inputs:
            %   name           - Name of the variation (e.g., 'speed', 'turn')
            %   varables       - Array of strings representing variable names involved in this variation 
            %                    (e.g., ["q0", "q1", "q2", "q3"] or ["u"])
            %   variation_data - Matrix where each row represents a different time instance or state,
            %                    and each column corresponds to a variable in 'varables'
            %   include_base   - Logical flag (true/false) indicating whether to include the base state 
            %                    as part of the variation
            %
            % Example:
            %   name = 'turn';
            %   varables = ["q0", "q1", "q2", "q3"];
            %   variation_data = [0.99, 0, 0, 0.08; 
            %                     0.98, 0, 0, 0.16];

            % `include_base` is not provided
            %if nargin < 3
            %    include_base = true;
            %end
            % Define a struct to store variation information
            variation.name = name;               % Name of the variation
            variation.varables = varables;       % Array of variable names as strings
            variation.variation_data = variation_data;  % Data associated with each variable
            %variation.include_base = include_base; % Bool used to determin if the base state is included in varation
            
            % Append the new variation struct to the variations array
            obj.variations(end + 1) = variation;
            obj.count_trim_points()
        end

        function count_trim_points(obj)
            num_trim_points_counter = 1;
            [~, num_vars] = size(obj.variations);
            for ii=1:num_vars
                variation = obj.variations(ii);
                [num_var, ~] = size(variation.variation_data);
                num_trim_points_counter = num_trim_points_counter * num_var;
            end
            obj.num_trim_points = num_trim_points_counter;
        end

        function generate_trim_batch(obj)
            % Generates a batch of trim points by creating combinations of each variation's states
            % Returns an array of trim points

            % Calculate total number of combinations (already stored in obj.num_trim_points)
            total_points = obj.num_trim_points;

            % Initialize the array of trim points
            trim_point_template = struct('name', [], 'id', [], 'states', obj.base_state);
            obj.trim_points = repmat(trim_point_template, total_points, 1);

            % Create the Cartesian product of all variation states
            combination_indices = obj.generate_combinations();

            % Generate each trim point based on the combination indices
            for point_idx = 1:total_points
                current_state = obj.base_state;
                
                trim_name = "";
                combo_index = combination_indices(point_idx, :);
                
                % Apply each variation to the current state
                for var_idx = 1:length(obj.variations)
                    
                    variation = obj.variations(var_idx);

                    %trim_name = trim_name + sprintf('%s %d, ', variation.name, point_idx);

                    row_idx = combination_indices(point_idx, var_idx);  % Row to use in variation data

                    trim_name = trim_name + sprintf('%s %d, ', variation.name, combo_index(var_idx));
                    
                    % Update current_state based on variation variables and data
                    for col_idx = 1:length(variation.varables)
                        state_index = obj.state_dict(variation.varables{col_idx});
                        current_state(state_index) = variation.variation_data(row_idx, col_idx);
                    end
                end

                obj.trim_points(point_idx).name = trim_name;
                obj.trim_points(point_idx).id = sprintf('%d.%d', obj.batch_id, point_idx);
                obj.trim_points(point_idx).states = current_state;
            end
        end
    end

    methods (Access = private)
        function combinations = generate_combinations(obj)
            % Generates the Cartesian product of row indices for each variation data matrix.
            % Each row in combinations represents one trim point configuration.
            
            num_variations = length(obj.variations);
            variation_sizes = zeros(1, num_variations);
            
            % Get the number of rows in each variation's data
            for i = 1:num_variations
                variation_sizes(i) = size(obj.variations(i).variation_data, 1);
            end
    
            % Generate the Cartesian product of all indices
            if num_variations == 1
                % If there's only one variation, simply create a column vector
                combinations = (1:variation_sizes(1))';
            else
                % For multiple variations, use ndgrid to generate combinations
                grid = cell(1, num_variations);
                [grid{:}] = ndgrid(1:variation_sizes(1), 1:variation_sizes(2:end));
                combinations = reshape(cat(num_variations + 1, grid{:}), [], num_variations);
            end
        end
    end

end
