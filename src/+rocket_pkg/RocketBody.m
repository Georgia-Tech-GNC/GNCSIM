
classdef RocketBody < handle
	properties
		body_params
		nose_cone_length {mustBeNumeric}
		ogive_radius    {mustBeNumeric}
		body_length     {mustBeNumeric}
		rocket_diameter {mustBeNumeric}
	end

	methods
		function obj = RocketBody(body_params)
			obj.body_params = body_params;
			obj.nose_cone_length = body_params.nose_cone_length;
			obj.ogive_radius = body_params.ogive_radius;
			obj.body_length = body_params.body_length;
			obj.rocket_diameter = body_params.rocket_diameter;
		end
        
        function [F_b, M_b] = get_forces_and_moments(obj, state, disturbances)
           % Computes aero forces and moments for body 
           F_b = [0, 0, 0];
           M_b = [0, 0, 0];
        end

		function compute_buildup(obj)
            n_nose_points = 20;  % Number of points to define the nose cone
            n_body_points = 2;   % Number of points to define the cylindrical body
            datcom_filename = 'BODY.dat';

            mach_numbers = [0.2, 0.4, 0.6];        % Array of Mach numbers to test
            altitudes_ft = [0, 5000, 10000];       % Array of altitudes in feet
            alphas_deg   = [-10, -8, -6, -4, -2, 0, 2, 4, 6, 8, 10]; % Array of angles of attack in degrees

            base_radius = obj.rocket_diameter / 2;
            total_length = obj.body_length + obj.nose_cone_length;
            
            % --- 1. Generate the Ogive Nose Cone Geometry ---
            L = obj.nose_cone_length;
            R = base_radius;
            
            rho_arc = (L^2 + R^2) / (2 * R); 
            %Rxc_offset = L - rho_arc;
            x_nose = linspace(0, L, n_nose_points);

            % Calculate the radius at each x-station using the circle equation
            r_nose = sqrt(rho_arc^2 - (x_nose - L).^2) + R - rho_arc;
            
            % Create x-stations for the body, starting where the nose cone ends
            x_body = linspace(L, total_length, n_body_points);
            r_body = ones(1, n_body_points) * R; % Radius is constant
            
            % Combine the arrays, removing the duplicate point at the junction
            body_stations_x = [x_nose(1:end-1), x_body];
            body_radii      = [r_nose(1:end-1), r_body];
            total_points    = length(body_stations_x);

            fileID = fopen(datcom_filename, 'w');
            if fileID == -1
                error('Could not open file for writing.');
            end

            if ~isempty(mach_numbers)
                fprintf(fileID, ' $FLTCON NMACH=%.1f, MACH(1)=%.4f,\n', length(mach_numbers), mach_numbers(1));
                % Loop through the rest of the Mach numbers, printing each on a new line
                for i = 2:length(mach_numbers)
                    fprintf(fileID, ' %.4f,\n', mach_numbers(i));
                end
            end
            
            % --- Write subsequent variables ---
            % Note: These do NOT start with '$FLTCON'
            if ~isempty(altitudes_ft)
                fprintf(fileID, ' NALT=%.1f, ALT(1)=%.4f,\n', length(altitudes_ft), altitudes_ft(1));
                for i = 2:length(altitudes_ft)
                    fprintf(fileID, ' %.4f,\n', altitudes_ft(i));
                end
            end
            
            if ~isempty(alphas_deg)
                fprintf(fileID, ' NALPHA=%.1f, ALSCHD(1)=%.4f,\n', length(alphas_deg), alphas_deg(1));
                % Loop to the SECOND-TO-LAST element, printing a comma
                for i = 2:length(alphas_deg)
                    fprintf(fileID, ' %.4f,\n', alphas_deg(i));
                end
            end
            fprintf(fileID, ' LOOP=2.0$\n');

            fprintf(fileID, ' $BODY   NX=%.1f, BNOSE=''OGIVE'',\n', total_points);

            % --- Write the X-stations array ---
            fprintf(fileID, ' X(1)=%.4f,', body_stations_x(1));
            for i = 2:total_points
                % Add a newline every 8 values for readability, matching the example
                if mod(i-2, 8) == 0 && i > 2
                    fprintf(fileID, '\n       ');
                end
                fprintf(fileID, ' %.4f,', body_stations_x(i));
            end
            % Remove the last comma and add a newline
            fseek(fileID, -1, 'cof'); % Move cursor back one character
            fprintf(fileID, '\n');

            % --- Write the Radii array ---
            fprintf(fileID, ' R(1)=%.4f,', body_radii(1));
            for i = 2:total_points
                if mod(i-2, 8) == 0 && i > 2
                    fprintf(fileID, '\n       ');
                end
                fprintf(fileID, ' %.4f,', body_radii(i));
            end
            fseek(fileID, -1, 'cof'); % Move cursor back one character
            fprintf(fileID, '$\n');

            fclose(fileID);

            if ispc
                % --- Windows ---
                executable_name = 'DATCOM\datcom_win.exe';
                if exist(executable_name, 'file') ~= 2
                    error('"%s" not found in the current directory.', executable_name);
                end
                command = sprintf('echo %s | %s', datcom_filename, executable_name);
            elseif isunix
                disp("imagine being on mac. could not be me")
            end

            status = system(command);
            disp('DATCOM execution finished.');
		end

		function draw_body(obj)
            figure;
            plot(body_stations_x, body_radii, 'k-');
            hold on;
            plot(body_stations_x, -body_radii, 'k-');
            axis equal;
            grid on;
		end
	end
end

