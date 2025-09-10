
classdef RocketDynamics
	properties
		body
		fins
		rocket_params
		motor_params
	end

	methods
		function obj = RocketDynamics(body, fins, rocket_params, motor_params)
			obj.body = body;
			obj.fins = fins;
			obj.rocket_params = rocket_params;
			obj.motor_params = motor_params;
		end

		function draw(obj)
			% TODO: draw rocket
		end
	end
end

