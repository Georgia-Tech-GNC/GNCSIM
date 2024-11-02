classdef TrimPoint < handle

    properties (SetAccess = private)
        file_path

        flight_data
        num_rows
        num_cols

        flight_data_num

        burn_time

        batches
        num_batches

        column_positions_csv
    end

    methods
        function obj = TrimPoint(file_path)

            % Set the class properties
            obj.file_path = file_path;

            % Create import options for readtable to handle custom formats
            opts = detectImportOptions(obj.file_path, 'ReadVariableNames', false);
            
            % Adjust import options to treat all columns as text (strings)
            opts = setvartype(opts, 'char');
            
            % Read the flight data from the CSV file into a table
            obj.flight_data = readtable(obj.file_path, opts);

            obj.flight_data_num = readmatrix(obj.file_path);
            obj.flight_data_num(any(isnan(obj.flight_data_num), 2), :) = [];

            % Required variables to check in the data
            required_vars = ["# Time (s)", "Total velocity (m/s)", "Thrust (N)"];

            % Get the size of the flight_data table
            [obj.num_rows, obj.num_cols] = size(obj.flight_data);

            %
            obj.column_positions_csv = look_for_data_types(obj, required_vars);

            % Throw an error if the required variables are not found
            if any(obj.column_positions_csv == -1)
                missingVarsString = strjoin(required_vars, ', ');
                error("Unable to find the required variables: %s", missingVarsString);
            end
        end

        function generate_trim_batches(obj, time_arr)
            obj.num_batches = length(time_arr);
            obj.batches = TrimBatch.empty(obj.num_batches, 0);
            for ii=1:obj.num_batches
                time = time_arr(ii);
                base_state = obj.get_vertical_state();
                [u, thrust] = obj.get_info(time);
                base_state(1) = u;
                base_state(13) = thrust;
                batch = TrimBatch(base_state, time, ii);
                obj.batches(ii) = batch;
            end
        end

        function add_u_variations(obj, time_arr, num_trim_points_per_time, trim_spacing)
            % add check if obj.num_batches == length(time_arr) 
            for ii=1:length(time_arr)
                u_vars = obj.generate_trim_points_from_u_var(time_arr, num_trim_points_per_time, trim_spacing);
                obj.batches(ii).add_variation("speed", "u", u_vars.')
            end
        end

        function add_custom_variations(obj, batch_indexes, name, var, data)
            for ii=1:length(batch_indexes)
                batch = obj.batches(batch_indexes(ii));
                batch.add_variation(name, var, data)
            end
        end

        function generate_trim_points(obj)
            for ii=1:obj.num_batches
                obj.batches(ii).generate_trim_batch()
            end
        end

        function u_vars = generate_trim_points_from_u_var(obj, time_arr, num_trim_points_per_time, trim_spacing)
            for ii=1:length(time_arr)
                [u, ~] = obj.get_info(time_arr(ii));

                lower_bound = u * (1 - trim_spacing);
                upper_bound = u * (1 + trim_spacing);
    
                exp_space = linspace(log(lower_bound), log(upper_bound), num_trim_points_per_time);
                
                u_vars = exp(exp_space);
            end
        end

        function [u, thrust] = get_info(obj, time)
            time_data = obj.flight_data_num(:, obj.column_positions_csv(1));
            speed_data = obj.flight_data_num(:, obj.column_positions_csv(2));
            thrust_data = obj.flight_data_num(:, obj.column_positions_csv(3));

            u = interp1(time_data, speed_data, time, 'linear');
            thrust = interp1(time_data, thrust_data, time, 'linear');
        end
        
        function event_time = find_event_time(obj, event_name)
            % Initialize event_time to NaN (in case the event is not found)
            event_time = NaN;

            % Iterate through all rows to find the event in the 4th column
            for row = 1:obj.num_rows
                for col = 1:obj.num_cols
                    % Get the current element in the 4th column
                    current_element = obj.flight_data{row, col};
                    
                    % Check if the current element contains the event name
                    if contains(current_element, event_name)
                        % Extract the time value from the string
                        event_time_str = extractBetween(current_element, "t=", " seconds");
                        event_time = str2double(event_time_str);
                        return;
                    end
                end
            end

            % If the event is not found, return NaN
            if isnan(event_time)
                warning('Event "%s" not found in the data.', event_name);
            end
        end

        function positions = look_for_data_types(obj, required_vars)

            % This is where the variable types are stored (e.g., row 4)
            row_num = 4;
            
            % Initialize positions array with -1 for all required variables
            positions = -1 * ones(1, length(required_vars));
            
            % Iterate over the columns of the flight data
            for col = 1:obj.num_cols
                % Get the current element in the specified row
                current_element = obj.flight_data{row_num, col};
                
                % Check if the current element is a member of the required variables
                for i = 1:length(required_vars)
                    if strcmp(current_element{1}, required_vars(i))
                        % If a match is found, store the column index
                        positions(i) = col;
                    end
                end
            end
        end
    end

    methods (Static)
        function full_state = get_vertical_state()
            % state = [u; v; w; p; q; r; q0; q1; q2; q3]
            state = [0; 0; 0; 0; 0; 0; 1; 0; 0; 0];

            % contol = [Roll; Yaw; Thrust]
            control = [0; 0; 0];
            
            % full state = [u; v; w; p; q; r; q0; q1; q2; q3; Roll; Yaw; Thrust]
            full_state = [state; control];
        end
    end
end