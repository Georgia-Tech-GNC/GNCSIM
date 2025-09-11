
classdef RocketFins
	properties
		fin_params
		num_fins
		fins  % array of fin objects
    end
    
	methods
		function obj = RocketFins(fin_params)
			obj.fin_params = fin_params;
			obj.num_fins = fin_params.num_fins;
		end

		function [F_b, M_b] = get_forces_and_momnets(obj, state, disturbances)
			F_b = [0, 0, 0];
			M_b = [0, 0, 0];
			for ii=1:obj.num_fins
				[F_b_fin, M_b_fin] = obj.fins(ii).get_forces_and_momnets(state, disturbances);
				F_b = F_b + F_b_fin;
				M_b = M_b + M_b_fin;
			end
		end
	end
end

