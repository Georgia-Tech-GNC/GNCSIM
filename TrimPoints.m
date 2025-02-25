% Main Authors: David Reynolds
classdef TrimPoints < handle
    % TrimPoint Class for Generating Trim Points
    % This class reads flight data from a CSV file, processes it, and generates trim points
    % based on specified times and variations. It manages batches of trim points and provides
    % methods to add custom variations and retrieve specific flight information.

    properties (Constant)
        % State Names
        % A constant cell array containing the names of all possible states.
        state_names = {"u", "v", "w", "p", "q", "r", ...
                      "q0", "q1", "q2", "q3", "Roll", "Yaw", "Thrust"};
        
        % State Indices
        % A constant array representing the indices corresponding to each state name.
        state_index_con = 1:length(TrimBatch.state_names);
        controller_ID = 'LQR';
    end

    properties (SetAccess = private)
        % The path to the CSV file containing the flight data.
        file_path
        
        % A table containing the raw flight data read from the CSV file.
        flight_data
        
        % Integers representing the size of the flight_data table.
        num_rows
        num_cols
        
        % A matrix containing the numerical values from the flight data,
        % with NaN rows removed.
        flight_data_num
        
        % An array of TrimBatch objects, each representing a batch of trim points
        % generated for specific times.
        batches
        
        % An integer representing the total number of batches.
        num_batches
        
        % An array of structs, each containing a trim point with its name, ID, and states.
        trim_points
        
        % An array of structs, each containing a trimmed point with its name, ID, states and cost.
        trimmed_points
        
        % An array indicating the column indices of required variables in the flight data.
        column_positions_csv

        % State Dictionary
        % A containers.Map object mapping state names (as strings) to their corresponding indices.
        state_dict  

        % An array of structs, each containing the reference state for
        % a second of controlled flight.
        reference_states

    end

    methods
        function obj = TrimPoints(file_path)
            % Constructor for TrimPoint Class
            % Initializes the TrimPoint object by reading and processing the flight data.
            %
            % Input:
            %   file_path - String representing the path to the CSV file containing flight data.
            
            % Set the file path property
            obj.file_path = file_path;
            
            % Create import options without relying on automatic detection
            opts = detectImportOptions(obj.file_path, 'ReadVariableNames', false, 'PreserveVariableNames', true);
            
            % Set all variable types to character arrays to avoid interpretation issues
            opts.VariableTypes(:) = {'char'};
            
            % Ensure no rows are automatically skipped
            opts.DataLines = [1, Inf]; % Read all rows
            
            % Read the flight data from the CSV file into a table
            obj.flight_data = readtable(obj.file_path, opts);
            
            % Read numerical data from the CSV file into a matrix
            obj.flight_data_num = readmatrix(obj.file_path);
            
            % Remove rows containing NaN values from flight_data_num
            obj.flight_data_num(any(isnan(obj.flight_data_num), 2), :) = [];
            
            % Define required variables to check in the data
            required_vars = ["# Time (s)", "Total velocity (m/s)", "Thrust (N)"];
            
            % Get the size of the flight_data table
            [obj.num_rows, obj.num_cols] = size(obj.flight_data);
            
            % Find the column positions of the required variables in the CSV file
            obj.column_positions_csv = obj.look_for_data_types(required_vars);
            
            % Throw an error if any of the required variables are not found
            if any(obj.column_positions_csv == -1)
                missingVarsString = strjoin(required_vars(obj.column_positions_csv == -1), ', ');
                error("Unable to find the required variables: %s", missingVarsString);
            end

            % Convert state names to character arrays for compatibility with containers.Map
            state_keys = cellfun(@char, TrimBatch.state_names, 'UniformOutput', false);
            state_values = num2cell(TrimBatch.state_index_con);
            
            % Create a dictionary (Map) with state names as keys and their indices as values
            obj.state_dict = containers.Map(state_keys, state_values);  
        end
        
        function [controller_ID] = get_controllerID(obj)
            controller_ID = obj.controller_ID;
        end

        function generate_trim_batches(obj, time_arr)
            % Generates Trim Batches for Specified Times
            % This method creates TrimBatch objects for each time in time_arr and initializes them
            % with the corresponding base states.
            %
            % Input:
            %   time_arr - Array of times at which to generate trim batches.
            
            % Set the number of batches based on the length of time_arr
            obj.num_batches = length(time_arr);
            
            % Initialize the batches array with empty TrimBatch objects
            obj.batches = TrimBatch.empty(obj.num_batches, 0);
            
            % Loop over each time to create and configure the TrimBatch
            for ii = 1:obj.num_batches
                time = time_arr(ii);
                
                % Get the vertical state as the base state
                base_state = TrimPoints.get_vertical_state();
                
                % Retrieve the velocity 'u' and thrust at the specified time
                [u, thrust] = obj.get_info(time);
                
                % Update the base_state with the retrieved 'u' and thrust values
                base_state(1) = u;       % Update 'u' in the state vector
                base_state(13) = thrust; % Update 'Thrust' in the control vector
                
                % Create a new TrimBatch with the updated base_state, time, and batch ID
                batch = TrimBatch(base_state, time, ii);
                
                % Add the batch to the batches array
                obj.batches(ii) = batch;
            end
        end

        function add_u_variations(obj, time_arr, num_trim_points_per_time, AerodynamicsObj, CG_t_handle, rho_t_handle)
            % Adds 'u' Variations to Each Batch Based on Specified Parameters
            % This method generates variations in 'u' (velocity) for each b atch and adds them.
            %
            % Inputs:
            %   time_arr                - Array of times corresponding to each batch.
            %   num_trim_points_per_time - Number of trim points to generate per time instance.
            %   trim_spacing            - Fractional spacing around the base 'u' value for variations.
            
            % Check if the number of batches matches the length of time_arr
            if obj.num_batches ~= length(time_arr)
                error('Number of batches does not match the length of time_arr.');
            end
            
            % Loop over each time to generate and add 'u' variations
            for ii = 1:length(time_arr)
                % Generate 'u' variations for the current time
                u_vars = obj.generate_trim_points_from_u_var(time_arr(ii), num_trim_points_per_time, AerodynamicsObj, CG_t_handle, rho_t_handle);
                
                % Add the 'u' variations to the corresponding batch
                obj.batches(ii).add_variation("speed", "u", u_vars.');
            end
        end

        function add_custom_variations(obj, batch_indexes, name, var, data, include_base_state)
            % Adds Custom Variations to Specified Batches
            % This method allows adding user-defined variations to selected batches.
            %
            % Inputs:
            %   batch_indexes - Array of batch indices to which the variation will be added.
            %   name          - Name of the variation.
            %   var           - Variable name(s) involved in the variation (as a string or array).
            %   data          - Variation data matrix corresponding to 'var'.
            %   include_base_state - Logical flag (true/false) to include base state in the variation.

            if nargin < 6
                include_base_state = false;
            end

            % Loop over each specified batch index
            for ii = 1:length(batch_indexes)
                % Get the batch at the current index
                batch = obj.batches(batch_indexes(ii));
                
                % copy data into new variable
                new_data = data;
                
                % If include_base_state is true, add base state to the variation data
                if include_base_state
                    % Initialize an array to store base state values for each variable in 'var'
                    base_state_values = zeros(1, length(var));
        
                    % Populate base_state_values by looking up each variable in the base state
                    for jj = 1:length(var)
                        var_name = var(jj);
                        var_index = batch.state_dict(var_name);  % Get the index of the variable
                        base_state_values(jj) = batch.base_state(var_index);  % Get the base state value
                    end
                    % Prepend base_state_values to the data matrix
                    new_data = [base_state_values; data];
                end

                % Add the custom variation to the batch
                batch.add_variation(name, var, new_data);
            end
        end

        function generate_trim_points(obj, rocket, env, flight, aero)
            % Generates Trim Points from All Batches and Combines Them
            % This method generates trim points for each batch and aggregates them into obj.trim_points.
            
            % Template for an individual trim point struct
            trim_point_template = struct('time', [], 'id', [], 'name', [], 'states', []);
            
            % Initialize total_trim_points counter
            total_trim_points = 0;
            
            % Generate trim batches and accumulate total number of trim points
            for ii = 1:obj.num_batches
                % Generate trim points for the current batch
                obj.batches(ii).generate_trim_batch();
                
                % Accumulate the total number of trim points
                total_trim_points = total_trim_points + obj.batches(ii).num_trim_points;
            end
            
            % Preallocate the trim_points array with the total number of trim points
            obj.trim_points = repmat(trim_point_template, total_trim_points, 1);
            
            % Index to keep track of the position in obj.trim_points
            current_index = 1;
            
            % Combine trim points from all batches into obj.trim_points
            for ii = 1:obj.num_batches
                % Get the trim points from the current batch
                batch_trim_points = obj.batches(ii).trim_points;
                
                % Determine the number of trim points in the current batch
                num_points_in_batch = obj.batches(ii).num_trim_points;
                
                % Assign the batch's trim points to the appropriate positions in obj.trim_points
                obj.trim_points(current_index:(current_index + num_points_in_batch - 1)) = batch_trim_points;
                
                % Update the current_index for the next iteration
                current_index = current_index + num_points_in_batch;
            end

            % perform trimming opimization
            obj.trim_rocket_model(rocket, env, flight, aero)
        end

        function batch_trim_points = get_trim_points(obj, target_time)
            % Finds the closest trim point in time to the specified target time
            %
            % Inputs:
            %   target_time - The time value to find the closest trim point to (double)
            %
            % Outputs:
            %   batch_trim_points - The trim points associated with the batch closest in time to target_time (structure or array)
            
            % Initialize an array to store the time values of each batch in obj.batches
            batch_times = zeros(1, length(obj.batches));
            
            % Populate batch_times with the time values from each batch
            for i = 1:length(obj.batches)
                batch_times(i) = obj.batches(i).time;  % Extract and store the time of the current batch
            end
            
            % Calculate the difference between each batch time and the target time
            time_differences = batch_times - target_time;
            
            % Find the absolute differences to measure the closeness to target_time
            abs_time_differences = abs(time_differences);
            
            % Identify the index of the batch with the smallest time difference (closest time)
            [~, closest_index] = min(abs_time_differences);
            
            % Retrieve the trim points associated with the batch closest to the target time
            batch_trim_points = obj.batches(closest_index).trim_points;
        end

        function export_trim_points_to_csv(obj, output_file_path)
            % Exports the Generated Trim Points to a CSV File
            %
            % This method exports the generated trim points into a CSV file.
            % Each state variable is placed in a separate column, along with
            % additional information such as 'name', 'id', and 'time'.
            %
            % Input:
            %   output_file_path - String representing the path where the CSV file will be saved.
            %
            % Example:
            %   trim_point.export_trim_points_to_csv('trim_points.csv');
            
            % Check if trim_points are generated
            if isempty(obj.trim_points)
                error('No trim points to export. Please generate trim points first.');
            end
            
            % Extract all state variable names from the first trim_point using state_dict
            % Assuming state_dict is a map with keys as variable names and values as indices

			state_indices = values(obj.state_dict, obj.state_names);
            state_indices = cell2mat(state_indices);

			% Sort the state_names based on their indices to maintain consistent order
    		[~, sorted_order] = sort(state_indices);
    		state_names_sorted = obj.state_names(sorted_order);

            % Number of state variables
            num_states = length(obj.state_names);
            
            % Create column headers: 'Name', 'ID', 'Time', followed by state variable names
            headers = [{"Time", "ID", "Name"}, state_names_sorted];
            
            % Initialize a cell array to hold all data
            num_trim_points = length(obj.trim_points);
            data_cell = cell(num_trim_points, length(headers));
            
            % Populate the data_cell with trim_points data
            for i = 1:num_trim_points
                trim_pt = obj.trim_points(i);
                
                % Extract 'name', 'id', and 'time'
                data_cell{i, 3} = trim_pt.name;
                data_cell{i, 2} = trim_pt.id;
                data_cell{i, 1} = trim_pt.time;
                
                % Extract state variables
                % Assuming 'states' is a vector corresponding to state_dict
                states = trim_pt.states;
                for j = 1:num_states
                    var_name = state_names_sorted{j};
                    var_index = obj.state_dict(var_name);
                    data_cell{i, 3 + j} = states(var_index);
                end
            end
            
            % Flatten the headers by extracting the contents of each nested cell
            headers = cellfun(@(x) x{1}, headers, 'UniformOutput', false);

            % Convert cell array to table
            trim_table = cell2table(data_cell, 'VariableNames', headers);
            
            % Write the table to CSV
            try
                writetable(trim_table, output_file_path);
                fprintf('Trim points successfully exported to %s\n', output_file_path);
            catch ME
                error('Failed to write trim points to CSV: %s', ME.message);
            end
        end

        function export_to_ref_h(obj, output_file_path)
            % Check if trim_points are generated
            if isempty(obj.trim_points)
                error('No trim points to export. Please generate trim points first.');
            end
           
            fid = fopen(output_file_path, 'w');
            if fid == -1
                error('Could not open file %s for writing.', output_file_path);
            end

            num_trim_points = length(obj.trim_points);
            start_time = obj.trimmed_points(1).time;
            for ii=1:num_trim_points
                trim_pt = obj.trimmed_points(ii);

                time = trim_pt.time - start_time;
                airspeed = trim_pt.states(1);
                q3 = trim_pt.states(10);
                K = trim_pt.K;
                %flat_K = K(:);
                flat_K = reshape(K.', 1, []);

                lineStr = sprintf('{ %.6f, { %.6f, %.6f }, { ', time, airspeed, q3);
        
                % Append each element of flat_K, separated by commas.
                numK = length(flat_K);
                for j = 1:numK
                    if j < numK
                        lineStr = sprintf('%s%.6f, ', lineStr, flat_K(j));
                    else
                        lineStr = sprintf('%s%.6f', lineStr, flat_K(j));
                    end
                end
                
                % Close the brace and add a trailing comma.
                lineStr = sprintf('%s } },\n', lineStr);
                
                % Write the line to the file.
                fprintf(fid, '%s', lineStr);
            end

            % Close the file.
            fclose(fid);
        end

        function u_vars = generate_trim_points_from_u_var(obj, time, num_trim_points_per_time, AerodynamicsObj, CG_t_handle, rho_t_handle)
            % Generates Variations in 'u' (Velocity) Around a Base Value
            % This method creates an array of 'u' values centered around
            % the base 'u' at a given time. Each 'u' value successively varies the
            % aerodynamic forces on the rocket by less than 8%.
            %
            % Inputs:
            %   time                     - The time at which to generate 'u' variations.
            %   num_trim_points_per_time - Number of 'u' variations to generate.
            %   aero                     - Aero object to access calcAeroBodyFrame_JV.
            %   CG_t_handle              - CG(t) function for calcAeroBodyFrame_JV.
            %   rho_t_handle             - rho(t) function for calcAeroBodyFrame_JV.
            %
            % Output:
            %   u_vars - Array of 'u' variation values.

            percent = 0.08;
            
            % Retrieve the base 'u' value at the specified time
            [u, ~] = obj.get_info(time);

            % Initializing u_vars array
            u_vars_temp = zeros(1, num_trim_points_per_time);

            % Determine the middle index
            if mod(num_trim_points_per_time, 2) == 0
                middle = num_trim_points_per_time / 2; % Choose the lower middle for even n
            else
                middle = ceil(num_trim_points_per_time / 2); % Standard middle for odd n
            end
            
            % Assign a value to the middle element
            u_vars_temp(middle) = u; % Assign a value to the chosen middle

            % Get initial aero forces
            [FAb, MAb]= AerodynamicsObj.calcAeroBodyFrame_JV(u, 0, 0, 0, 0, rho_t_handle(time), CG_t_handle(time));

            % Initializing u_iterative and u_step for loop
            u_iterative = u;
            u_step = 0.1;
            eps = 1E-8;
            
            % Loop forward from the middle to the end
            for i = middle+1:num_trim_points_per_time
                % Calculating initial magnitude of aero force and moment
                FAb_initial_mag = norm(FAb);
                MAb_initial_mag = norm(MAb);

                % Initializing percent change in aero force and moment
                percent_change_FAb = 0;
                percent_change_MAb = 0;

                % Increasing u until 8% change is reached
                while (percent_change_FAb < percent && percent_change_MAb < percent)
                    u_iterative = u_iterative + u_step;
                    [FAb, MAb]= AerodynamicsObj.calcAeroBodyFrame_JV(u_iterative, 0, 0, 0, 0, rho_t_handle(time), CG_t_handle(time));
                    percent_change_FAb = abs((norm(FAb) - FAb_initial_mag) / (FAb_initial_mag + eps));
                    percent_change_MAb = abs((norm(MAb) - MAb_initial_mag) / (MAb_initial_mag + eps));
                end
                u_vars_temp(i) = u_iterative;
            end

            % Get initial aero forces
            [FAb, MAb]= AerodynamicsObj.calcAeroBodyFrame_JV(u, 0, 0, 0, 0, rho_t_handle(time), CG_t_handle(time));

            % Initializing u_iterative and u_step for loop
            u_iterative = u;
            u_step = -0.1;
            
            % Loop backward from the middle to the beginning
            for i = middle-1:-1:1
                % Calculating initial magnitude of aero force and moment
                FAb_initial_mag = norm(FAb);
                MAb_initial_mag = norm(MAb);

                % Initializing percent change in aero force and moment
                percent_change_FAb = 0;
                percent_change_MAb = 0;

                % Increasing u until 8% change is reached
                while (percent_change_FAb < percent && percent_change_MAb < percent)
                    u_iterative = u_iterative + u_step;
                    [FAb, MAb]= AerodynamicsObj.calcAeroBodyFrame_JV(u_iterative, 0, 0, 0, 0, rho_t_handle(time), CG_t_handle(time));
                    percent_change_FAb = abs((norm(FAb) - FAb_initial_mag) / (FAb_initial_mag + eps));
                    percent_change_MAb = abs((norm(MAb) - MAb_initial_mag) / (MAb_initial_mag + eps));
                    %percent_change_MAb = abs((norm(MAb) - MAb_initial_mag) / MAb_initial_mag);
                end
                u_vars_temp(i) = u_iterative;
            end

            % Returning final u_vars for given batch
            u_vars = u_vars_temp;
        end

        function [u, thrust] = get_info(obj, time)
            % Retrieves 'u' (Velocity) and Thrust at a Specific Time
            % This method interpolates the flight data to find 'u' and thrust values at the given time.
            %
            % Input:
            %   time - The time at which to retrieve 'u' and thrust.
            %
            % Outputs:
            %   u      - Interpolated velocity value at the specified time.
            %   thrust - Interpolated thrust value at the specified time.
            
            % Extract time, velocity, and thrust data from flight_data_num using column positions
            time_data = obj.flight_data_num(:, obj.column_positions_csv(1));
            speed_data = obj.flight_data_num(:, obj.column_positions_csv(2));
            thrust_data = obj.flight_data_num(:, obj.column_positions_csv(3));
            
            % Interpolate 'u' (velocity) at the specified time
            u = interp1(time_data, speed_data, time, 'linear');
            
            % Interpolate thrust at the specified time
            thrust = interp1(time_data, thrust_data, time, 'linear');
        end
        
        function event_time = find_event_time(obj, event_name)
            % Finds the Time of a Specific Event in the Flight Data
            % This method searches for a specified event in the flight data and extracts its time.
            %
            % Input:
            %   event_name - The name of the event to search for (as a string).
            %
            % Output:
            %   event_time - The time at which the event occurs. Returns NaN if not found.
            
            % Initialize event_time to NaN in case the event is not found
            event_time = NaN;
            
            % Iterate through all rows and columns to find the event
            for row = 1:obj.num_rows
                for col = 1:obj.num_cols
                    % Get the current element in the table
                    current_element = obj.flight_data{row, col};
                    
                    % Check if the current element contains the event name
                    if contains(current_element{1}, event_name)
                        % Extract the time value from the string (assuming format "t=... seconds")
                        event_time_str = extractBetween(current_element{1}, "t=", " seconds");
                        event_time = str2double(event_time_str);
                        return;  % Exit the function once the event is found
                    end
                end
            end
            
            % If the event is not found, display a warning
            if isnan(event_time)
                warning('Event "%s" not found in the data.', event_name);
            end
        end

        function positions = look_for_data_types(obj, required_vars)
            % Finds Column Positions of Required Variables in the Flight Data
            % This method searches for specific variable names in the flight data to determine their column indices.
            %
            % Input:
            %   required_vars - Array of variable names to search for.
            %
            % Output:
            %   positions - Array of column indices corresponding to the required variables.
            %               If a variable is not found, its position is set to -1.
            
            % row num is the row with all header information
            row_num = 1;

            % loop over all rows until a # is found
            for row=1:obj.num_rows

                % gets first column of every row
                current_element = obj.flight_data{row, 1};

                % perform string comparison
                if strcmp(current_element, "#")

                    % header information is after the single '#'
                    row_num = row + 1;

                    % break out of loop after header information is found
                    break;
                end
            end
            
            % Initialize positions array with -1 for all required variables
            positions = -1 * ones(1, length(required_vars));
            
            % Iterate over the columns of the flight data
            for col = 1:obj.num_cols
                % Get the current element in the specified row and column
                current_element = obj.flight_data{row_num, col};
                
                % Check if the current element matches any of the required variables
                for i = 1:length(required_vars)
                    if strcmp(current_element{1}, required_vars(i))
                        % If a match is found, store the column index
                        positions(i) = col;
                    end
                end
            end
        end

        function trim_rocket_model(obj, rocket, env, flight, aero)
            % trim_rocket_model Processes Trim Points to Optimize Rocket Model States

            % Define a reference quaternion representing the "up" orientation
            up = [1, 0, 0, 0];  % Quaternion components (q0, q1, q2, q3)

            % Number of constraints used in optimization
            num_constraints = 18;

            % Initialize trimmed_points by copying the existing trim_points
            obj.trimmed_points = obj.trim_points;
        
            % Determine the total number of trim points to process
            total_points = numel(obj.trim_points);

            last_percent = 0;  % Initialize the last printed percent
            
            for ii = 1:total_points
                % Extract the current trim point's state vector
                trim_point = obj.trim_points(ii).states;
                
                % Extract the time associated with the current trim point
                t = obj.trim_points(ii).time;
        
                % Calculate the current progress percentage based on the iteration index
                current_percent = floor((ii / total_points) * 100);
        
                % Display progress if the current percentage exceeds the last recorded percentage
                if current_percent > last_percent
                    fprintf('Trimming Progress: %d%%\n', current_percent);
                    last_percent = current_percent;  % Update the last_percent to the current_percent
                end

                one_vec = ones(1, num_constraints);
                H = diag(one_vec);
                if ~isequal(trim_point(7:10), up)  % This is for turning
                    H(4,4) = 100;
                    H(6,6) = 100;
                    H(7,7) = 100;
                    H(8,8) = 100;
                    H(9,9) = 100;
                    H(10,10) = 100;
                end
        
                % Perform optimization to find the optimal state vector (ZStar) that minimizes the cost function
                [ZStar, f0] = fminsearch(@(Zguess) TrimPoints.cost_rocket_model(Zguess, trim_point, t, rocket, env, flight, aero, H), ...
                                          trim_point, ...
                                          optimset('TolX', 1e-10, 'MaxFunEvals', 100, 'MaxIter', 100, 'Display', 'off'));
                
                % Update the trimmed_points array with the optimized cost and state vector
                obj.trimmed_points(ii).cost = f0;     % Store the minimized cost value
                obj.trimmed_points(ii).states = ZStar; % Store the optimized state vector
            end
            fprintf("Finished Trimming %d Trim Points\n", total_points)
        end

        function generate_K(obj, straight_up_Q, straight_up_R, maneuver_Q, maneuver_R, compute_A, compute_B)
            % generate_K Generates LQR Gain Matrices for Each Trimmed Point
            % Inputs:
            %   straight_up_Q   - State weighting matrix for straight-up flight
            %   straight_up_R   - Control weighting matrix for straight-up flight
            %   maneuver_Q      - State weighting matrix for maneuvering flight
            %   maneuver_R      - Control weighting matrix for maneuvering flight
            %   compute_A       - System matrix function handel
            %   compute_B       - Control matrix function handel

            % Load necessary structures from the master_struct.mat file
            %load("master_struct.mat")
            
            % Determine the total number of trim points to process
            total_points = numel(obj.trim_points);

            % Define a reference quaternion representing the "up" orientation
            up = [1, 0, 0, 0];  % Quaternion components (q0, q1, q2, q3)

            % Print Statment
            fprintf("Generating K Matrices\n")

            % Iterate over each trim point to compute and assign the LQR gain matrices
            for ii = 1:total_points
                % Extract the time associated with the current trim point
                t = obj.trim_points(ii).time;
                
                % Extract the optimized state vector from the trimmed points
                ZStar = obj.trimmed_points(ii).states;
        
                % Compute the system matrix A and B based on the optimized state and time
                A = compute_A(ZStar(1), ZStar(2), ZStar(3), ZStar(4), ZStar(5), ZStar(6), ...
                             ZStar(7), ZStar(8), ZStar(9), ZStar(10), t);

                A = real(A);  % Fix to make A matrix real
                B = compute_B(t);
                
                % Determine whether the current state corresponds to "straight up" flight
                if isequal(ZStar(7:10), up)  % Compare the quaternion components to the "up" reference
                    Q = straight_up_Q;
                    R = straight_up_R;
                else
                    Q = maneuver_Q;
                    R = maneuver_R;
                end
                
                % Compute the LQR gain matrix K using the system matrices A and B, and weighting matrices Q and R
                K = lqr(A, B, Q, R);

                % Assign the computed matrices and gain to the corresponding trimmed point
                obj.trimmed_points(ii).A = A;    % Store the system matrix A
                obj.trimmed_points(ii).B = B;    % Store the input matrix B
                obj.trimmed_points(ii).Q = Q;    % Store the state weighting matrix Q
                obj.trimmed_points(ii).R = R;    % Store the control weighting matrix R
                obj.trimmed_points(ii).K = K;    % Store the LQR gain matrix K
            end
        end
       
        function draw_u_trim_points(obj, start_time, end_time)
            % Plots the Speed ("u") Trim Points as a Function of Time Alongside Real "u" Data
            %
            % This method extracts the real 'u' data from the flight data and the 'u' values
            % from the generated trim points within the specified time range. It then plots
            % both on the same graph for comparison.
            %
            % Inputs:
            %   start_time - The start time of the plotting range (double).
            %   end_time   - The end time of the plotting range (double).
            %
            % Example:
            %   trim_point.plot_u_trim_points(0, 13);
            
            % Validate input times
            if start_time >= end_time
                error('start_time must be less than end_time.');
            end
                        
            % Create a time vector with specified start_time and end_time
            plot_time = linspace(start_time, end_time);
            
            % Use get_info to get 'u' values at the plot_time
            [u_real, ~] = obj.get_info(plot_time);
            
            % Initialize arrays to hold trim points' 'u' and their corresponding times within the range
            u_trim = [];
            trim_times = [];
            
            % Iterate through each batch to collect trim points and their associated times
            for ii = 1:obj.num_batches
                batch = obj.batches(ii);
                batch_time = batch.time;
                
                for jj = 1:length(batch.variations)
                    if batch.variations(jj).varables == "u"
                        u_var = batch.variations(jj).variation_data;
                        trim_times = [trim_times, repmat(batch_time, length(u_var), 1)];
                        u_trim = [u_trim, u_var];
                    end
                end
            end
            
            % Ensure that trim_times and u_trim are column vectors
            trim_times = trim_times(:);
            u_trim = u_trim(:);
            
            % Check if trim_times and u_trim have the same length
            if length(trim_times) ~= length(u_trim)
                error('trim_times and u_trim must have the same number of elements.');
            end
            
            % Create a new figure for the plot
            figure;
            
            % Plot the real 'u' data as a blue line
            plot(plot_time, u_real, 'b-', 'LineWidth', 2);
            hold on;
            
            % Plot the trim points' 'u' values as red filled circles
            scatter(trim_times, u_trim, 50, 'ro', 'filled');
            
            % Label the plot axes
            xlabel('Time (s)', 'FontSize', 12);
            ylabel('Speed (u) (ft/s)', 'FontSize', 12);
            
            % Add a title to the plot
            title('Speed (u) Trim Points vs Time', 'FontSize', 14);
            
            % Add a legend to differentiate between real data and trim points
            legend('Real u Data', 'Trim Points', 'Location', 'best');
            
            % Enable grid for better readability
            grid on;
            
            % Release the hold on the current figure
            hold off;
        end

        function [K, x0, u0] = LQR_configuration_selector(obj, x, t)
            % Author: Karsten Caillet

            % LQR_configuration_selector Selects the appropriate LQR configuration based on the current state and time.
            %
            % INPUTS:
            % obj - The object containing trimmed_points and reference_states structures with pre-computed data.
            % x   - Current state vector (9x1) consisting of:
            %       [u, v, w, p, q, r, q1, q2, q3], where:
            %           u, v, w: Velocity components (m/s)
            %           p, q, r: Angular velocity components (rad/s)
            %           q1, q2, q3: Quaternion components representing orientation.
            % t   - Current time (scalar, in seconds). The function rounds `t` to the nearest integer.
            %
            % OUTPUTS:
            % K   - LQR gain matrix corresponding to the closest matching trimmed condition.
            % x0  - Reference state vector corresponding to the matched condition.
            % u0  - Reference control vector corresponding to the matched condition.
    
            % Variables from input
            u = x(1);
            v = x(2);
            w = x(3);
            q1 = x(7);
            q2 = x(8);
            q3 = x(9);
        
            % Set t to a nonzero integer (1-12)
            t = round(t);
            if (t < 1.0)
                t = 1.0;
            end
        
            % Get airspeed
            airspeed = sqrt(u^2 + v^2 + w^2);
        
            % Find the matching time index in trimmed_points struct
            t_values = round([obj.trimmed_points.time]);  % Extract all t values into an array
            ind = (t_values == t);  % Compare t_values to t
    
            % Check if any time matches
            if ~any(ind)
                K = zeros(size([obj.trimmed_points(1).K]));
                x0 = zeros(size(x));
            end
        
            % Calculate the differences for the matching rows
            speedDiff = arrayfun(@(s) abs(sqrt(s.states(1)^2 + s.states(2)^2 + s.states(3)^2) - airspeed), obj.trimmed_points(ind));
            q1Diff = arrayfun(@(s) abs(s.states(8) - q1), obj.trimmed_points(ind));
            q2Diff = arrayfun(@(s) abs(s.states(9) - q2), obj.trimmed_points(ind));
            q3Diff = arrayfun(@(s) abs(s.states(10) - q3), obj.trimmed_points(ind));
            distance = sqrt(speedDiff.^2 + q1Diff.^2 + q2Diff.^2 + q3Diff.^2);
        
            % Find the minimum distance and its index
            [~, min_index] = min(distance);
        
            % Extract the corresponding K gain
            K_sublist = {obj.trimmed_points(ind).K};
            K = K_sublist{min_index};

            % Find the matching time index in reference_state struct
            t_values = round([obj.reference_states.time]);  % Extract all t values into an array
            ind = (t_values == t);  % Compare t_values to t
            
            % Construct x0 (reference state vector)
            reference_states_index = logical([1 1 1 1 1 1 0 1 1 1 0 0 0]); % Getting u, v, w, p, q, r, q1, q2, q3
            x0 = obj.reference_states(ind).states(reference_states_index);  % Getting reference states for the correct second
            
            % Construct u0 (reference control vector)
            u0 = obj.reference_states(ind).states(end-2:end); % Getting reference controls for the correct second
        end

        function del_u = compute_controls(obj, K, del_x)
            del_u = -K*del_x;
        end

        function obj = set_reference_states(obj, time_arr, variations_cell_arr)
            % Author: Karsten Caillet

            % Sets Reference States Based on Time Array and Variations
            %
            % This method initializes the reference states for the object based on the
            % provided `time_arr` and `variations_cell_arr`. Each reference state is created
            % using a predefined template and adjusted based on the variations provided.
            %
            % Inputs:
            %   time_arr            - Array of time values for the reference states (1D numeric array).
            %   variations_cell_arr - Cell array of variations to modify the reference states.
            %                         Format: {{time, {'state', value}, {'state', value}, ...}, ...}

            % Template for an individual reference_state struct
            reference_state_template = struct('time', [], 'states', []);
            
            % Initialize total_reference_states counter
            total_reference_states = length(time_arr);
            
            % Preallocate the reference_state array with the total number of reference states
            obj.reference_states = repmat(reference_state_template, total_reference_states, 1);

            % Populate reference states with default values
            for i = 1:length(time_arr)
                obj.reference_states(i).time = time_arr(i);
                obj.reference_states(i).states = TrimPoints.get_vertical_state();
            end

            % Iterate over each variation entry to adjust the reference states
            for i = 1:length(variations_cell_arr)
                % Find the index of the variation time in the time array
                variation_index = find(time_arr == variations_cell_arr{i}{1}, 1);
                if isempty(variation_index)
                    error('No matching time found for variation.');
                end
                
                % Apply each variation to the appropriate state
                for j = 2:length(variations_cell_arr{i})
                    state_index = obj.state_dict(variations_cell_arr{i}{j}{1});
                    obj.reference_states(variation_index).states(state_index) = variations_cell_arr{i}{j}{2};
                end
            end        
        end

        function reference_state = get_reference_state(obj, time)
            % Author: Karsten Caillet

            % Retrieves the Reference State for a Specified Time
            %
            % This method searches the reference states for the given time and
            % returns the corresponding state. An error is raised if the time 
            % is not found in the reference states.
            %
            % Inputs:
            %   time - The time value for which the reference state is requested (double).
            %
            % Output:
            %   reference_state - The state corresponding to the specified time.

            % Validate that the given time exists in the reference state array
            if ~ismember(time, [obj.reference_states.time])
                % Raise an error if no matching time is found
                assert(false,'ERROR: No matching time found in the reference state array\n');
                return;
            end

            % Find the index corresponding to the given time
            index = find([obj.reference_states.time] == time);

            % Retrieve the reference state at the found index
            reference_state = obj.reference_states(index).states;
        end    
    end

    methods (Static)
        function full_state = get_vertical_state()
            % Generates the Default Vertical State of the Rocket
            % This static method returns a state vector representing a vertical rocket with zero velocities and default orientations.
            %
            % Output:
            %   full_state - A column vector representing the full state of the rocket.
            %
            % State Vector Components:
            %   - u, v, w: Linear velocities in body axes (m/s)
            %   - p, q, r: Angular velocities (rad/s)
            %   - q0, q1, q2, q3: Quaternion components representing orientation
            %   - Roll, Yaw, Thrust: Control inputs
            
            % State variables (u; v; w; p; q; r; q0; q1; q2; q3)
            state = [0; 0; 0; 0; 0; 0; 1; 0; 0; 0];
            
            % Control variables (Roll; Yaw; Thrust)
            control = [0; 0; 0];
            
            % Combine state and control into the full state vector
            full_state = [state; control];
        end

        function f0 = cost_rocket_model(Z, trim_point, t, rocket, environment, flight, aero, H)
            % cost_rocket_model Evaluates the Cost Function for Rocket Trim Optimization
            % Inputs:
            %   Z            - Current state vector (including control variables).
            %                  Format: [u; v; w; p; q; r; q0; q1; q2; q3; M_roll; M_yaw; T].
            %   trim_point   - Desired trim point state vector for comparison.
            %                  Format: [u_trim; v_trim; w_trim; ...; T_trim].
            %   t            - Current time (used for dynamic calculations).
            %   rocket       - Rocket object containing physical properties and parameters.
            %   environment  - Environment object defining atmospheric conditions.
            %   flight       - Flight dynamics object for simulation.
            %   aero         - Aerodynamic model object for the rocket.
            %   H            - Weighting matrix for penalizing deviations in constraints.
            %
            % Output:
            %   f0 - Cost value computed as the weighted sum of squared deviations from
            %        the desired trim point and dynamic constraints.

            % Extract state variables from the input vector Z
            u = Z(1);   
            v = Z(2);   
            w = Z(3);   
            p = Z(4);   
            q = Z(5);   
            r = Z(6);   
            q0 = Z(7);  
            q1 = Z(8);  
            q2 = Z(9);  
            q3 = Z(10); 

            % Extract corresponding trim point values
            u_trim = trim_point(1);
            v_trim = trim_point(2);
            w_trim = trim_point(3);
            p_trim = trim_point(4);
            q_trim = trim_point(5);
            r_trim = trim_point(6);
            q0_trim = trim_point(7);
            q1_trim = trim_point(8);
            q2_trim = trim_point(9);
            q3_trim = trim_point(10);
            M_roll_trim = trim_point(11);
            M_yaw_trim = trim_point(12);
            T_trim = trim_point(13);
            
            % Extract control variables from the input vector Z
            M_roll = Z(11); 
            M_yaw = Z(12);  
            T = Z(13);

            % Compute the rocket dynamics using the current state (Z)
            X = [0, 0, 0, Z(1:10)]; % State vector (position not included)
            U = Z(11:13);           % Control vector (moments and thrust)
            [xdot, ~] = rocketModel(t, X, rocket, environment, flight, aero, -1);
            xdot = real(xdot);      % Ensure dynamics are real-valued

            % Extract derivatives from the rocket dynamics
            udot = xdot(4);
            vdot = xdot(5);
            wdot = xdot(6);
            pdot = xdot(11);
            qdot = xdot(12);
            rdot = xdot(13);

             % Define constraint deviations for optimization
            num_constraints = 18; % Let the number of constraints happen here
            Q = ones(num_constraints, 1);
            Q(1) = u - u_trim;
            Q(2) = 0; % v;
            Q(3) = w - w_trim;
            Q(4) = p - p_trim;
            Q(5) = q - q_trim;
            Q(6) = r - r_trim;
            Q(7) = q0 - q0_trim;
            Q(8) = q1 - q1_trim;
            Q(9) = q2 - q2_trim;
            Q(10) = q3 - q3_trim;
            Q(11) = M_roll - M_roll_trim;
            Q(12) = M_yaw - M_yaw_trim;
            Q(13) = T - T_trim;
            Q(14) = 0; % vdot;
            Q(15) = wdot;
            Q(16) = pdot;
            Q(17) = qdot;
            Q(18) = sqrt(q0^2 + q1^2 + q2^2 + q3^2) - 1;

            f0 = Q' * H * Q; % Compute the cost function
        end
    end
end
