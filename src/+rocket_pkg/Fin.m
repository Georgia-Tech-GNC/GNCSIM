classdef Fin < handle
	% FIN Represents an aerodynamic fin component on a rocket or aircraft

	properties
		fin_params		% Structure containing geometric and placement parameters for this fin
        name			% String identifier for the fin (e.g., 'fin_1', 'fin_2', etc.)
		aero			% DATCOM aerodynamic data
	end

	methods
		% Constructor
		function obj = Fin(name, fin_params)
            obj.name = name;
            obj.fin_params = fin_params;
        end

        function [F_b, M_b] = get_forces_and_moments(obj, state, disturbances)

           % Rotation from inertial to body using quaternion
           q = state(8:11);
           C_B_I = Utils.dir_cosine(q);

           % Body frame velocity
           v_b = state(5:7);

           % v_w = C_B_I * disturbances.wind
           v_w = C_B_I * [0;0;0]; % Wind velocity (ignored for now)
           
           v_b = v_b + v_w;

           % Rotation from body frame to component (fin) frame
		   C_C_B = Utils.fin_dcm(0, obj.fin_params.angle);

           % Component frame velocity and ang vel
           w_b = state(12:14);
           r_cb = -obj.fin_params.translation;
           v_c = C_C_B*(v_b + cross(w_b, r_cb));
			
		   % Get component forces and moments based on 
		   [F_c, M_c] = rocket_pkg.DatcomManager.get_forces_and_moments(state, v_c, obj.aero);

           % Converting back to body frame
           C_B_C = C_C_B';
           F_b = C_B_C*F_c;
           M_b_ross = cross(r_cb.', F_c);
           M_b = C_B_C*M_c + M_b_ross(:);
		end
	end
end