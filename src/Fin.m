classdef Fin < handle
	properties
		fin_params
        is_prime  {boolean}
        name
        num_alpha
		alpha  % range of alpha values
		mach  % range of mach numbers
		aero  % DATCOM cell array of aerodynamic data
		symmetric_fin  % This is a variable that holds the primary component
		% If is_prime is True then symmetric_comp is empty. If is_prime is
		% False then symmetric_fin is type Fin that holds the symmetric Fin
	end
	methods
		% Constructor for the Fin class
		function obj = Fin(name, is_prime, fin_params)
			
            obj.name = name;
            obj.is_prime = is_prime;
            obj.fin_params = fin_params;
			
			% number of angle of attack points
			obj.num_alpha = 11;
			obj.alpha = linspace(-15, 15, obj.num_alpha);
        end

        function [F_b, M_b] = get_forces_and_moments(obj, state, disturbances)
           % Computes forces and moments for body 
           F_b = [0, 0, 0];
           M_b = [0, 0, 0];
        end

		
	end
end