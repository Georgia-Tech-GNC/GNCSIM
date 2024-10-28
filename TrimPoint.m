classdef TrimPoint

    properties (SetAccess = private)
        file_path

        flight_data
        num_rows
        num_cols

        flight_data_num

        burn_time
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
            positions = look_for_data_types(obj, required_vars);

            % Throw an error if the required variables are not found
            if any(positions == -1)
                missingVarsString = strjoin(required_vars, ', ');
                error("Unable to find the required variables: %s", missingVarsString);
            end
        end

        function generate_trim_points(obj, time_arr, num_trim_points_per_time)
            disp("X")
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
end