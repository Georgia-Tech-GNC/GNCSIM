classdef Fin < handle
	properties
		fin_params
        is_prime  {boolean}
        name
        num_alpha
		alpha  % range of alpha values
		mach  % range of mach numbers
		aero  % DATCOM cell array of aerodynamic data
		% symmetric_fin  % This is a variable that holds the primary component
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
           q = state(8:11);
           Utils.dir_cosine_matlab(q)

           % Body frame velocity
           v_i = state(5:7);
           v_w = [0;0;0]; % Wind velocity (ignored for now)
           v_b = R_ib*(v_i + v_w);

           % Rotation from body frame to component (fin) frame
           delta = obj.fin_params.angle;
           R_bc = [1, 0, 0; 0, cosd(delta), -sind(delta); 0, sind(delta, cosd(delta))];

           % Component frame velocity and ang vel
           w_b = state(12:14);
           r_bc = [0; (obj.fin_params.root_chord/2)*cosd(delta); (obj.fin_params.root_chord/2)*sind(delta)];
           v_c = R_bc*(v_b + cross(w_b, r_bc));
           w_c = R_bc*w_b;

           % Aerodynamic coefficients
           v_a = sqrt(v_c(1)^2+v_c(2)^2+v_c(3)^2);
           a = 343; % Speed of sound approximation (TODO: Replace with better approximation)
           M = v_a/a;
           AoA = atan2d(v_c(3),v_c(1));
           beta = asind(v_c(2)/v_a);
           
           pos = state(2:4);
           altitude = pos(3);  % z componet of position
           [cL, cY, cD, cl, cm, cn] = aero_interp(M, AoA, altitude);

           S = obj.aero.sref;
           b = obj.aero.blref;
           c = obj.aero.cbar;
           q = v_a * S;

           % Rotation from wind frame to component
           R_wc(1,:) = [cosd(AoA)*cosd(beta), -cosd(AoA)*sind(beta), -sind(AoA)];
           R_wc(2,:) = [sind(beta), cosd(beta), 0];
           R_wc(3,:) = [cosd(beta)*sind(AoA), -sind(AoA)*sind(beta), cosd(AoA)];

           % Forces + Moments in component frame
           L = -cL*q*S;
           Y = cY*q*S;
           D = -cD*q*S;
           F_w = [D; Y; L];
           F_c = R_wc * F_w;

           l = cl*q*S*b;
           m = cm*q*S*c;
           n = cn*q*S*b;
           M_c = [l; m; n];

           % Converting back to body frame
           R_cb = R_bc';
           F_b = R_cb*F_c;
           M_b = R_cb*M_c;
        end

        function [cL, cY, cD, cl, cm, cn] = aero_interp(mach, AoA_deg, alt_m)
            alt_ft = alt_m / 0.3048;
            cL = interp3(obj.aero.mach, obj.aero.alpha, obj.aero.alt, obj.aero.cL, ...
                              mach, AoA_deg, alt_ft, 'linear', 'extrap');
            cY = interp3(obj.aero.mach, obj.aero.alpha, obj.aero.alt, obj.aero.cY, ...
                              mach, AoA_deg, alt_ft, 'linear', 'extrap');
            cD = interp3(obj.aero.mach, obj.aero.alpha, obj.aero.alt, obj.aero.cD, ...
                              mach, AoA_deg, alt_ft, 'linear', 'extrap');
            cl = interp3(obj.aero.mach, obj.aero.alpha, obj.aero.alt, obj.aero.cl, ...
                              mach, AoA_deg, alt_ft, 'linear', 'extrap');
            cm = interp3(obj.aero.mach, obj.aero.alpha, obj.aero.alt, obj.aero.cm, ...
                              mach, AoA_deg, alt_ft, 'linear', 'extrap');
            cn = interp3(obj.aero.mach, obj.aero.alpha, obj.aero.alt, obj.aero.cn, ...
                              mach, AoA_deg, alt_ft, 'linear', 'extrap');
        end
	end
end