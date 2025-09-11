
classdef RocketBody < handle
	properties
		params
		nose_cone_length {mustBeNumeric}
		ogive_radius {mustBeNumeric}
		body_length {mustBeNumeric}
		rocket_diameter {mustBeNumeric}
	end

	methods
		function obj = RocketBody(params)
			obj.params = params;
			obj.nose_cone_length = params.nose_cone_length;
			obj.ogive_radius = params.ogive_radius;
			obj.body_length = params.body_length;
			obj.rocket_diameter = params.rocket_diameter;
		end
        
        function [F_b, M_b] = get_forces_and_moments(obj, state, disturbances)
           % Computes forces and moments for body 
           F_b = [0, 0, 0];
           M_b = [0, 0, 0];
        end

		function compute_buildup(obj)
		end

	end
end

