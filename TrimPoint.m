classdef TrimPoint

    properties (SetAccess = private)
        file_path
        time_arr
        num_trim_points_per_time

        flight_data
        num_rows
        num_cols
    end

    methods
        function obj = TrimPoint(file_path, time_arr, num_trim_points_per_time)

            % Set the class properties
            obj.file_path = file_path;
            obj.time_arr = time_arr;
            obj.num_trim_points_per_time = num_trim_points_per_time;

            % Create import options for readtable to handle custom formats
            opts = detectImportOptions(obj.file_path, 'ReadVariableNames', false);
            
            % Adjust import options to treat all columns as text (strings)
            opts = setvartype(opts, 'char');
            
            % Read the flight data from the CSV file into a table
            obj.flight_data = readtable(obj.file_path, opts);

            % Required variables to check in the data
            required_vars = ["# Time (s)", "Total velocity (m/s)", "Thrust (N)"];

            % Get the size of the flight_data table
            [obj.num_rows, obj.num_cols] = size(obj.flight_data);

            % Throw an error if the required variables are not found
            if ~obj.look_for_data_types(required_vars)
                missingVarsString = strjoin(required_vars, ', ');
                error("Unable to find the required variables: %s", missingVarsString);
            end
        end
        
        function all_members = look_for_data_types(obj, required_vars)

            % This is where the varable types are stored
            row_num = 4;

            all_members = true;

            for col = 1:obj.num_cols
                % Get the current element
                current_element = obj.flight_data{row_num, col};
                
                % Check if the current element is a member of the search strings
                if ~any(ismember(required_vars, current_element{1}))
                    % If a match is found, display the row, column, and string
                    all_members = false;
                    %disp(['Match found at row ', num2str(row), ', column ', num2str(col), ': ', currentElement]);
                end
            end
        end
    end
end