
%% NOTE: May not need this

classdef AeroComponent < handle
	properties
		name  {string}
		is_prime {boolean}
	end
	methods
		function obj = AeroComponent(name, is_prime)
			obj.name = name;
			obj.is_prime = is_prime;
		end

		function [alpha, beta] = get_alpha_beta(v)
			% TODO
		end
	end
end