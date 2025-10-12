
classdef RocketBody < rocket_pkg.DatcomManager
	properties
		body_params
		body_outline  % in for of 2xn where first row is x value and y is radius
		nose_cone_length {mustBeNumeric}
		ogive_radius    {mustBeNumeric}
		body_length     {mustBeNumeric}
		rocket_diameter {mustBeNumeric}
		aero
	end

	methods
		function obj = RocketBody(body_params, flight_con_params)
			obj = obj@rocket_pkg.DatcomManager(flight_con_params)

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

		function r_nose = get_nose_radii(obj, x_nose)
			L = obj.nose_cone_length;
            R = obj.rocket_diameter / 2;

			rho_arc = (L^2 + R^2) / (2 * R);

			% Calculate the radius at each x-station using the circle equation
			r_nose = sqrt(rho_arc^2 - (x_nose - L).^2) + R - rho_arc;
		end

		function compute_buildup(obj)
            n_nose_points = 20;  % Number of points to define the nose cone
            n_body_points = 2;   % Number of points to define the cylindrical body
            datcom_filename = 'BODY.dat';
            
            total_length = obj.body_length + obj.nose_cone_length;
            
            x_nose = linspace(0, obj.nose_cone_length, n_nose_points);
			r_nose = obj.get_nose_radii(x_nose);            
            
            % Create x-stations for the body, starting where the nose cone ends
            x_body = linspace(obj.nose_cone_length, total_length, n_body_points);
            r_body = ones(1, n_body_points) * obj.rocket_diameter / 2; % Radius is constant
            
            % Combine the arrays, removing the duplicate point at the junction
            body_stations_x = [x_nose(1:end-1), x_body];
            body_radii      = [r_nose(1:end-1), r_body];
            total_points    = length(body_stations_x);
			obj.body_outline = [body_stations_x; body_radii];

            obj.open_datcom(datcom_filename)

			obj.add_flight_condition()

            fprintf(obj.fileID, ' $BODY   NX=%.1f, BNOSE=''OGIVE'',\n', total_points);

            % --- Write the X-stations array ---
            fprintf(obj.fileID, ' X(1)=%.4f,', body_stations_x(1));
            for i = 2:total_points
                % Add a newline every 8 values for readability, matching the example
                if mod(i-2, 8) == 0 && i > 2
                    fprintf(obj.fileID, '\n       ');
                end
                fprintf(obj.fileID, ' %.4f,', body_stations_x(i));
            end
            % Remove the last comma and add a newline
            fseek(obj.fileID, -1, 'cof'); % Move cursor back one character
            fprintf(obj.fileID, '\n');

            % --- Write the Radii array ---
            fprintf(obj.fileID, ' R(1)=%.4f,', body_radii(1));
            for i = 2:total_points
                if mod(i-2, 8) == 0 && i > 2
                    fprintf(obj.fileID, '\n       ');
                end
                fprintf(obj.fileID, ' %.4f,', body_radii(i));
            end
            fseek(obj.fileID, -1, 'cof'); % Move cursor back one character
            fprintf(obj.fileID, '$\n');

            obj.close_datcom()
            obj.run_datcom(datcom_filename)

			% Import DATCOM Data
            aero_cell = datcomimport('datcom.out');
            obj.aero = aero_cell{1};
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

