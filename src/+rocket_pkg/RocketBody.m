
classdef RocketBody < rocket_pkg.DatcomManager
	% ROCKETBODY Represents the aerodynamic body of a rocket

	properties
		body_params			% Structure containing body geometry parameters
		body_outline		% [2 x n] array: row 1 = x stations, row 2 = body radii
		aero				% DATCOM aerodynamic data
	end

	methods
		% Constructor
		function obj = RocketBody(body_params, flight_con_params)
			% Call superclass constructor (required for inheritance)
			obj = obj@rocket_pkg.DatcomManager(flight_con_params)

			obj.body_params = body_params;
		end
        
        function [F_b, M_b] = get_forces_and_moments(obj, state, disturbances)
           	% Computes aerodynamic forces and moments on the body

           	% Rotation from inertial frame to body frame using quaternion
           	q = state(8:11);
           	R_ib = Utils.dir_cosine(q);
	
           	% Compute body-frame velocity
           	v_i = state(5:7);
           	v_w = [0;0;0];   % Placeholder for wind
           	v_b = R_ib*(v_i + v_w);
			
			% Compute aerodynamic forces and moments using DATCOM data
			[F_b, M_b] = Utils.get_forces_and_moments(state, v_b, obj.aero);
		end

		function r_nose = get_nose_radii(obj, x_nose)
			% Computes nose cone radius distribution

			L = obj.body_params.nose_cone_length;
            R = obj.body_params.rocket_diameter / 2;
			
			% Compute the radius of curvature for the ogive
			rho_arc = (L^2 + R^2) / (2 * R);

			% Circle-based profile equation for an ogive nose
			r_nose = sqrt(rho_arc^2 - (x_nose - L).^2) + R - rho_arc + 0.0001;
		end

		function compute_buildup(obj)
			% Generates DATCOM input file for the rocket body

            n_nose_points = 10;   % Number of points defining the nose cone
            n_body_points = 5;    % Number of points defining the cylindrical body
            datcom_filename = 'BODY.dat';
			xcg = 0.6;           % [m] Center of gravity location (temporary value)
            
			% Generate body geometry
            total_length = obj.body_params.body_length + obj.body_params.nose_cone_length;
            
			% Nose cone geometry
            x_nose = linspace(0, obj.body_params.nose_cone_length, n_nose_points);
			r_nose = obj.get_nose_radii(x_nose);            
            
            % Cylindrical body geometry
            x_body = linspace(obj.body_params.nose_cone_length, total_length, n_body_points);
            r_body = ones(1, n_body_points) * obj.body_params.rocket_diameter / 2; % Radius is constant
            
            % Combine nose cone and body into continuous profile
            body_stations_x = [x_nose(1:end-1), x_body];
            body_radii      = [r_nose(1:end-1), r_body];
            total_points    = length(body_stations_x);
			obj.body_outline = [body_stations_x; body_radii];
			
			% Open DATCOM input file and write sections
            obj.open_datcom(datcom_filename)
			obj.add_flight_condition()
            
			% Write $OPTINS Namelist
			R = obj.body_params.rocket_diameter / 2;
            fprintf(obj.fileID, ' $OPTINS SREF=%.4f, CBARR=%.4f, BLREF=%.4f$\n', pi*R^2, total_length, 2*R);

            % Write $SYNTHS Namelist
            fprintf(obj.fileID, ' $SYNTHS XW=%.4f, XCG=%.4f$\n', 0, xcg);

            fprintf(obj.fileID, ' $BODY   NX=%.1f,\n', total_points);

            % Write the X-stations array 
            fprintf(obj.fileID, ' X(1)=%.4f,', body_stations_x(1));
            for i = 2:total_points
                % Add a newline every 8 values for readability, matching the example
                if mod(i-2, 8) == 0 && i > 2
                    fprintf(obj.fileID, '\n');
                end
                fprintf(obj.fileID, ' %.4f,', body_stations_x(i));
			end
            fprintf(obj.fileID, '\n');

            % Write the Radii array
            fprintf(obj.fileID, ' R(1)=%.4f,', body_radii(1));
            for i = 2:total_points
                if mod(i-2, 8) == 0 && i > 2
                    fprintf(obj.fileID, '\n');
                end
                fprintf(obj.fileID, ' %.4f,', body_radii(i));
            end
            fseek(obj.fileID, -1, 'cof'); % Move cursor back one character
            fprintf(obj.fileID, '$\n');

			% Close DATCOM file
            obj.close_datcom();
            
			% Run the .exe DATCOM file
			obj.run_datcom(datcom_filename)

			% Import DATCOM Data
            aero_cell = datcomimport('datcom.out');
            obj.aero = aero_cell{1};
		end

		function draw_body(obj)
            % Draws a 3D rocket with a tangent ogive nose cone and cylindrical body.

            % Derived quantities
            r = obj.body_params.rocket_diameter/2;     % body radius
            x_nose = linspace(0, obj.body_params.nose_cone_length, 100);
            
            % Ogive profile (tangent ogive formula)
            % Based on geometry of circle with radius ogive_radius offset so tangent at base
            x_offset = obj.body_params.ogive_radius - sqrt(Rn^2 - r^2);
            y_profile = sqrt(obj.body_params.ogive_radius^2 - (obj.body_params.ogive_radius - x_nose - x_offset).^2);
            
            % Add cylindrical body after nose
            x_body = linspace(obj.body_params.nose_cone_length, obj.body_params.nose_cone_length + obj.body_params.body_length, 20);
            y_body = r * ones(size(x_body));
            
            % Combine profiles
            x = [x_nose, x_body];
            y = [y_profile, y_body];
            
            % Create 3D surface of revolution
            theta = linspace(0, 2*pi, 100);
            X = repmat(x(:), 1, 100);
            Y = repmat(y(:), 1, 100).*cos(theta);
            Z = repmat(y(:), 1, 100).*sin(theta);
            
            % Plot
            figure;
            surf(X, Y, Z, 'FaceColor', [0.8 0.8 0.9], 'EdgeColor', 'none');
            hold on;
            surf(X, -Y, Z, 'FaceColor', [0.8 0.8 0.9], 'EdgeColor', 'none'); % mirror
            axis equal;
            xlabel('Length (m)');
            ylabel('Radius (m)');
            zlabel('Z (m)');
            title(sprintf('Rocket: Ln=%.2f, Rn=%.2f, Lb=%.2f, D=%.2f', Ln, Rn, Lb, D));
            camlight headlight; lighting gouraud;
            grid on;
        end
	end
end

