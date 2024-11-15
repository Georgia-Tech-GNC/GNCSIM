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

            % Convert state names to character arrays for compatibility with containers.Map
            state_keys = cellfun(@char, TrimBatch.state_names, 'UniformOutput', false);
            state_values = num2cell(TrimBatch.state_index_con);
            
            % Create a dictionary (Map) with state names as keys and their indices as values
            obj.state_dict = containers.Map(state_keys, state_values);  
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
            obj.trim_rocket_model()
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

        function trim_rocket_model(obj)
            rocket = Rocket();
            flight = Flight();

            flight.set_simTime(13);
            flight.set_tStep(0.01);
            flight.set_tSpan();

            tSpan = flight.get_tSpan();
            [~, len_tSpan] = size(tSpan);
            uSpan = zeros(len_tSpan, 3);
            flight.set_uSpan(uSpan);
            
            %% THIS SHOULD BE CHANGED!!!

            % INSERT NEW MONTE CARLO PARAM (READ FROM EXCEL SHEET) HERE
            wetMass = 38.87;% update to read from excel sheet
            dryMass = 30.96; % update to read from excel sheet
            peakThrust = 2262; % update to read from excel sheet
            burnTime = 13.1; % update to read from excel sheet
            wetRollMOI = 0.2109; % update to read from excel sheet
            wetPitchMOI = 21.1212; % update to read from excel sheet
            wetYawMOI = 21.1202; % update to read from excel sheet
            dryRollMOI = 0.2036; % update to read from excel sheet
            dryPitchMOI = 18.99; % update to read from excel sheet
            dryYawMOI = 18.988; % update to read from excel sheet

            CG0 = 2.433; % initial CG from tip of nose (m)
            finalCG = 2.34; % final CG from tip of nose (m)
            length = 3.4; % length of rocket (m)
            diameter = 0.203; % m

            thrustCurve = [0.0, 0.457288;
            0.152, 0.314574;
            0.162, 40.2069;
            0.164, 148.3954;
            0.168, 598.904;
            0.172, 890.058;
            0.176, 995.096;
            0.178, 1158.031;
            0.182, 1342.306;
            0.19, 1556.716;
            0.222, 1959.8;
            0.234, 2050.53;
            0.254, 2111.01;
            0.308, 2119.9;
            0.356, 2073.29;
            0.47, 2091.07;
            0.528, 1989.543;
            0.582, 1825.825;
            0.648, 1733.526;
            0.782, 1658.07;
            0.892, 1701.725;
            0.914, 1633.231;
            0.966, 1669.558;
            1.7839, 1594.076;
            1.8239, 1551.156;
            3.1999, 1533.43;
            5.4538, 1378.986;
            6.9577, 1170.933;
            10.9216, 424.692;
            13.5895, 106.0499;
            14.6754, 12.60914;
            16.1354, 0.0;];

            rocket.set_wetMass(wetMass);
            rocket.set_dryMass(dryMass);
            rocket.set_wetMOI(diag([wetRollMOI, wetPitchMOI, wetYawMOI]));
            rocket.set_dryMOI(diag([dryRollMOI, dryPitchMOI, dryYawMOI]));
            rocket.set_initialCG(CG0);
            rocket.set_finalCG(finalCG);
            rocket.set_length(length)
            rocket.set_diameter(diameter)
            rocket.set_radius(diameter/2)

            rocket.set_burnTime(burnTime);
            rocket.set_thrustCurve(thrustCurve)

            %timmed_struct = struct("time", [], "id", 0, "name", "", "states", [], "cost", 0);

            obj.trimmed_points = obj.trim_points;
            
            total_points = numel(obj.trim_points);
            last_percent = 0;  % Initialize the last printed percent
            
            for ii = 1:total_points
                trim_point = obj.trim_points(ii).states;
                t = obj.trim_points(ii).time;
            
                % Calculate the current percentage
                current_percent = floor((ii / total_points) * 100);
            
                % Display progress if the percent has increased
                if current_percent > last_percent
                    fprintf('Trimming Progress: %d%%\n', current_percent);
                    last_percent = current_percent;
                end
            
                [ZStar, f0] = fminsearch(@(Zguess) obj.cost_rocket_model_turning(Zguess, trim_point, t, rocket, flight), trim_point, ...
                    optimset('TolX', 1e-10, 'MaxFunEvals', 100, 'MaxIter', 100, 'Display', 'off'));
            
                obj.trimmed_points(ii).cost = f0;
                obj.trimmed_points(ii).states = ZStar;
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

        function f0 = cost_rocket_model_turning(Z, trim_point, t, rocket, flight)
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
            
            M_roll = Z(11); 
            M_yaw = Z(12);  
            T = Z(13);

            X = [0,0,0, Z(1:10)];  % State variables now start at Z(1)

            U = Z(11:13);  % Control variables now start at Z(11)
            

            [xdot, ~] = rocketModel(t, X, rocket, flight, -1);
            xdot = real(xdot);

            udot = xdot(4);
            vdot = xdot(5);
            wdot = xdot(6);
            pdot = xdot(11);
            qdot = xdot(12);
            rdot = xdot(13);
            q0dot = xdot(7);
            q1dot = xdot(8);
            q2dot = xdot(9);
            q3dot = xdot(10);

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

            one_vec = ones(1, num_constraints);
            H = diag(one_vec); % Should be dimensions of the number of constraints
            H(4,4) = 100;
            H(6,6) = 100;
            H(7,7) = 100;
            H(8,8) = 100;
            H(9,9) = 100;
            H(10,10) = 100;

            f0 = Q' * H * Q; % Compute the cost function
        end 
    end
end
