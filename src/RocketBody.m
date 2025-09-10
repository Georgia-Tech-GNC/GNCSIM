
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

	end
end

