classdef TrimPoint < handle
    % TrimPoint Class for Generating Trim Points
    % This class reads flight data from a CSV file, processes it, and generates trim points
    % based on specified times and variations. It manages batches of trim points and provides
    % methods to add custom variations and retrieve specific flight information.
    
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
        
        % The time when the burn event occurs, extracted from the flight data.
        burn_time
        
        % An array of TrimBatch objects, each representing a batch of trim points
        % generated for specific times.
        batches
        
        % An integer representing the total number of batches.
        num_batches
        
        % An array of structs, each containing a trim point with its name, ID, and states.
        trim_points
        
        % An array indicating the column indices of required variables in the flight data.
        column_positions_csv
    end

    methods
        function obj = TrimPoint(file_path)
            % Constructor for TrimPoint Class
            % Initializes the TrimPoint object by reading and processing the flight data.
            %
            % Input:
            %   file_path - String representing the path to the CSV file containing flight data.
            
            % Set the file path property
            obj.file_path = file_path;
            
            % Create import options for readtable to handle custom formats
            opts = detectImportOptions(obj.file_path, 'ReadVariableNames', false);
            
            % Adjust import options to treat all columns as character arrays
            opts = setvartype(opts, 'char');
            
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
                base_state = obj.get_vertical_state();
                
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

        function add_u_variations(obj, time_arr, num_trim_points_per_time, trim_spacing)
            % Adds 'u' Variations to Each Batch Based on Specified Parameters
            % This method generates variations in 'u' (velocity) for each batch and adds them.
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
                u_vars = obj.generate_trim_points_from_u_var(time_arr(ii), num_trim_points_per_time, trim_spacing);
                
                % Add the 'u' variations to the corresponding batch
                obj.batches(ii).add_variation("speed", "u", u_vars.');
            end
        end

        function add_custom_variations(obj, batch_indexes, name, var, data)
            % Adds Custom Variations to Specified Batches
            % This method allows adding user-defined variations to selected batches.
            %
            % Inputs:
            %   batch_indexes - Array of batch indices to which the variation will be added.
            %   name          - Name of the variation.
            %   var           - Variable name(s) involved in the variation (as a string or cell array).
            %   data          - Variation data matrix corresponding to 'var'.
            
            % Loop over each specified batch index
            for ii = 1:length(batch_indexes)
                % Get the batch at the current index
                batch = obj.batches(batch_indexes(ii));
                
                % Add the custom variation to the batch
                batch.add_variation(name, var, data);
            end
        end

        function generate_trim_points(obj)
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
            first_trim_point = obj.trim_points(1);
            state_dict = obj.batches(1).state_dict;  % Assuming state_dict is the same across all batches
            state_names = keys(state_dict);

			state_indices = values(state_dict, state_names);
            state_indices = cell2mat(state_indices);

			% Sort the state_names based on their indices to maintain consistent order
    		[~, sorted_order] = sort(state_indices);
    		state_names_sorted = state_names(sorted_order);

            % Number of state variables
            num_states = length(state_names);
            
            % Create column headers: 'Name', 'ID', 'Time', followed by state variable names
            headers = [{'Time', 'ID', 'Name'}, state_names_sorted];
            
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
                    var_index = state_dict(var_name);
                    data_cell{i, 3 + j} = states(var_index);
                end
            end
            
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

        function u_vars = generate_trim_points_from_u_var(obj, time, num_trim_points_per_time, trim_spacing)
            % Generates Variations in 'u' (Velocity) Around a Base Value
            % This method creates an array of 'u' values centered around the base 'u' at a given time.
            %
            % Inputs:
            %   time                     - The time at which to generate 'u' variations.
            %   num_trim_points_per_time - Number of 'u' variations to generate.
            %   trim_spacing             - Fractional spacing around the base 'u' value.
            %
            % Output:
            %   u_vars - Array of 'u' variation values.
            
            % Retrieve the base 'u' value at the specified time
            [u, ~] = obj.get_info(time);
            
            % Calculate the lower and upper bounds for 'u' variations
            lower_bound = u * (1 - trim_spacing);
            upper_bound = u * (1 + trim_spacing);
            
            % Generate logarithmically spaced 'u' values between the bounds
            exp_space = linspace(log(lower_bound), log(upper_bound), num_trim_points_per_time);
            u_vars = exp(exp_space);
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
            
            % Row number where variable names are stored (e.g., row 4)
            row_num = 4;
            
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
    end
end
