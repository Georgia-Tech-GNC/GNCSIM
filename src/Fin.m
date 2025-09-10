classdef Fin < AeroComponent
	properties
		fin_params {double}
		alpha  % range of alpha values
		mach  % range of mach numbers
		aero  % DATCOM cell array of aerodynamic data
		is_prime  % Flag to indicate if this is a primary component
		symmetric_fin  % This is a variable that holds the primary component
		% If is_prime is True then symmetric_comp is empty. If is_prime is
		% False then symmetric_fin is type Fin that holds the symmetric Fin
	end
	methods
		% Constructor for the Fin class
		function obj = Fin(name, is_prime, fin_params)
			% Call the superclass (AeroComponent)
			obj@AeroComponent(name, is_prime);
			
			% Now you can initialize the subclass's own properties
			obj.fin_params = fin_params;
			
			% number of angle of attack points
			obj.num_alpha = 11;
			obj.alpha = linspace(-15, 15, obj.num_alpha);
			
		end

		function compute_buildup(obj, mach)
			obj.mach = mach;
		end
	end
end