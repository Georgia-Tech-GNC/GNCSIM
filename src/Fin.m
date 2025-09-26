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
		% Constructor
		function obj = Fin(name, is_prime, fin_params)
			
            obj.name = name;
            obj.is_prime = is_prime;
            obj.fin_params = fin_params;
			
			% number of angle of attack points
			obj.num_alpha = 11;
			obj.alpha = linspace(-15, 15, obj.num_alpha);
        end

        function [F_b, M_b, F_c, M_c] = get_forces_and_moments(obj, state, disturbances)
           % Computes forces and moments for body 
           F_c = [0, 0, 0];
           M_c = [0, 0, 0];

           % Rotation from inertial to body using quaternion
           q_ib = state(3);
           R_ib(1,:) = [2*(q_ib(1)^2+q_ib(2)^2)-1, 2*(q_ib(2)*q_ib(3)-q_ib(1)*q_ib(4)), 2*(q_ib(2)*q_ib(4)+q_ib(1)*q_ib(3))];
           R_ib(2,:) = [2*(q_ib(2)+q_ib(3)), 2*(q_ib(1)^2*q_ib(3)^2)-1, 2*(q_ib(3)*q_ib(4)-q_ib(1)*q_ib(2))];
           R_ib(3,:) = [2*(q_ib(2)*q_ib(4)-q_ib(1)*q_ib(3)), 2*(q_ib(3)*q_ib(4)+q_ib(1)*q_ib(2)), 2*(q_ib(1)^2+q_ib(4)^2)];

           % Body frame velocity
           v_i = state(2);
           v_w = [0;0;0]; % Wind velocity (ignored for now)
           v_b = R_ib*(v_i + v_w);

           % Rotation from body frame to component (fin) frame
           delta = obj.fin_params.angle;
           R_bc = [1, 0, 0; 0, cosd(delta), -sind(delta); 0, sind(delta, cosd(delta))];

           % Component frame velocity and ang vel
           w_b = state(4);
           r_bc = [0; (obj.fin_params.root_chord/2)*cosd(delta); (obj.fin_params.root_chord/2)*sind(delta)];
           v_c = R_bc*(v_b + cross(w_b, r_bc));
           w_c = R_bc*w_b;

           % Aerodynamic coefficients
           v_a = sqrt(v_c(1)^2+v_c(2)^2+v_c(3)^2);
           a = 343; % Speed of sound approximation (TODO: Replace with better approximation)
           M = v_a/a;
           alpha = atan2d(v_c(3),v_c(1));
           beta = asind(v_c(2)/v_a);
           % TODO: Check if below coefficients are accessed correctly, and
           % if helper aero function is needed.
           C_L = obj.aero.cL;
           C_Y = obj.aero.cY;
           C_D = obj.aero.cD;
           C_l = obj.aero.cl;
           C_m = obj.aero.cm;
           C_n = obj.aero.cn;
           S = obj.aero.sref;
           b = obj.aero.blref;
           c = obj.aero.cbar;
           q = v_a * S;

           % Rotation from wind frame to component
           R_wc(1,:) = [cosd(alpha)*cosd(beta), -cosd(alpha)*sind(beta), -sind(alpha)];
           R_wc(2,:) = [sind(beta), cosd(beta), 0];
           R_wc(3,:) = [cosd(beta)*sind(alpha), -sind(alpha)*sind(beta), cosd(alpha)];

           % Forces + Moments in component frame
           L = -C_L*q*S;
           Y = C_Y*q*S;
           D = -C_D*q*S;
           F_w = [D; Y; L];
           F_c = R_wc * F_w;

           l = C_l*q*S*b;
           m = C_m*q*S*c;
           n = C_n*q*S*b;
           M_c = [l; m; n];

           % Converting back to body frame
           R_cb = R_bc';
           F_b = R_cb*F_c;
           M_b = R_cb*M_c;
        end
		
	end
end