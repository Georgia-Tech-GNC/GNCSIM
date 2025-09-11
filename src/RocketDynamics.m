
classdef RocketDynamics
	properties
		body
		fins
		rocket_params
		motor_params
		controller 
	end

	methods
		function obj = RocketDynamics(body, fins, rocket_params, motor_params)
			obj.body = body;
			obj.fins = fins;
			obj.rocket_params = rocket_params;
			obj.motor_params = motor_params;
        end

        function [F_b, M_b] = get_forces_and_moments(obj, state, disturbances)
            % computes forces and moments 
            [F_b_body, M_b_body] = obj.body.get_forces_and_moments(state, disturbances);
            [F_b_fins, M_b_fins] = obj.fins.get_forces_and_moments(state, disturbances);
            F_b = F_b_body + F_b_fins;
            M_b = M_b_body + M_b_fins;

        end

		function draw(obj)
			% TODO: draw rocket
		end
	end
end

