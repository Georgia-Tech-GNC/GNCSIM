classdef RocketDynamics
	properties
		body
		fins
		rocket_params
		motor_params
		controller 

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
            % TODO: Calculate mass and moi
            % https://phys.libretexts.org/Bookshelves/Classical_Mechanics/Variational_Principles_in_Classical_Mechanics_(Cline)/13%3A_Rigid-body_Rotation/13.08%3A_Parallel-Axis_Theorem
        end

        function [F_b, M_b] = get_forces_and_moments(obj, state, disturbances)
            % computes forces and moments 
            [F_b_body, M_b_body] = obj.body.get_forces_and_moments(state, disturbances);
            [F_b_fins, M_b_fins] = obj.fins.get_forces_and_moments(state, disturbances);
            %[F_b_controller, M_b_controller] = obj.controller.get_forces_and_moments(state, disturbances, obj.motor_params);
            F_b = F_b_body + F_b_fins; %+ F_b_controller;
            M_b = M_b_body + M_b_fins; %+ M_b_controller;
        end

        function moi = get_moi(obj, mass)
            moi = mass * eye(3);
        end

		function draw(obj)
			% TODO: draw rocket
		end
	end
end

