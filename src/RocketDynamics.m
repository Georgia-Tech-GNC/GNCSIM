classdef RocketDynamics < handle
	properties
		body
		fins
		rocket_params
		motor_params
		controller 
        sensors

        mass_wet
        mass_dry
        moi_wet
        moi_dry
	end

	methods
		function obj = RocketDynamics(body, fins, rocket_params, motor_params)
			obj.body = body;
			obj.fins = fins;
			obj.rocket_params = rocket_params;
			obj.motor_params = motor_params;
        end

        function calc_mass_moi(obj)
            % TODO: Calculate mass and moi based on geometry
            % https://phys.libretexts.org/Bookshelves/Classical_Mechanics/Variational_Principles_in_Classical_Mechanics_(Cline)/13%3A_Rigid-body_Rotation/13.08%3A_Parallel-Axis_Theorem
        end

        function [F_b, M_b] = get_forces_and_moments(obj, state, disturbances, time)
            % TODO: Calculate CG based on time (below is just an idea)
            
            body_length = obj.aero.blref;
            CG_i = 2.433; % Add this and CG_f as inputs with thrust in master_script?
            CG_f = 2.34;
            dur = 13.1; % Also input this with thrust data?
            CG = ((CG_f - CG_i)/dur)*(t) + CG_i;
            
            % computes aero forces and moments 
            [F_b_body, M_b_body] = obj.body.get_forces_and_moments(state, disturbances, CG);
            [F_b_fins, M_b_fins] = obj.fins.get_forces_and_moments(state, disturbances, CG);

            %TODO: sensor_data = sensor.get_sensor_data(time, state, disturbances, CG)
            %TODO: state_estimate = navigation.get_state_estimate(state, disturbances)
            %TODO: [F_b_controller, M_b_controller] = obj.controller.get_forces_and_moments(state, disturbances, obj.motor_params);

            F_b = F_b_body + F_b_fins; %+ F_b_controller;
            M_b = M_b_body + M_b_fins; %+ M_b_controller;
        end

        function moi = get_moi(obj, mass)
            moi = mass * eye(3);
        end

		function draw(obj)
			% TODO: draw rocket (rough draft below, need to get access to rocket size parameters)
            
            figure; hold on; axis equal
            xlabel('X'); ylabel('Y'); zlabel('Z');
            view(3); grid on
            
            % Body (cylinder)
            [Xb, Yb, Zb] = cylinder(D_body/2, 50);
            Zb = Zb * L_body;
            surf(Xb, Yb, Zb, 'FaceColor', [0.8 0.8 0.9], 'EdgeColor', 'none');
            
            % Nose (cone)
            [Xn, Yn, Zn] = cylinder([D_body/2 0], 50);
            Zn = Zn * L_nose + L_body;
            surf(Xn, Yn, Zn, 'FaceColor', [0.9 0.7 0.7], 'EdgeColor', 'none');
            
            % Fins
            theta = linspace(0, 2*pi, n_fins+1);
            for i = 1:n_fins
                % Fin coordinates in XY-plane
                finX = [L_body-0.1*L_body, L_body-0.1*L_body, L_body-L_fin];
                finY = [D_body/2, D_body/2+H_fin, D_body/2];
                finZ = [0, 0, 0];
                
                % Rotate fin around Z-axis
                R = [cos(theta(i)) -sin(theta(i)); sin(theta(i)) cos(theta(i))];
                finCoords = R * [finY; finZ];
                
                % Draw fin (patch)
                patch(finX, finCoords(1,:), finCoords(2,:), [0.6 0.6 0.6]);
            end
            camlight; lighting gouraud
        end

        function compute_buildup(obj)
            %obj.body.compute_buildup()
            obj.fins.compute_buildup()
        end

	end
end

