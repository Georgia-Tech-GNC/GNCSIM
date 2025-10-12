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
           R_ib = Utils.dir_cosine(q);

           % Body frame velocity
           v_i = state(5:7);
           v_w = [0;0;0]; % Wind velocity (ignored for now)
           v_b = R_ib*(v_i + v_w);

           % Rotation from body frame to component (fin) frame
		   R_bc = Utils.fin_dcm(0, obj.fin_params.angle);

           % Component frame velocity and ang vel
           w_b = state(12:14);
           r_cb = -obj.fin_params.translation;
           v_c = R_bc*(v_b + cross(w_b, r_cb));
			
		   % Get component forces and moments based on 
		   [F_c, M_c] = Utils.get_forces_and_moments(state, v_c, obj.aero);

           % Converting back to body frame
           R_cb = R_bc';
           F_b = R_cb*F_c;
           M_b = R_cb*M_c;
		end
	end
end