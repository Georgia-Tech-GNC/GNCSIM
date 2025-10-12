classdef Fin < handle
	properties
		fin_params
        name
        num_alpha
		aero  % DATCOM cell array of aerodynamic data
	end
	methods
		% Constructor
		function obj = Fin(name, fin_params)
			
            obj.name = name;
            obj.fin_params = fin_params;
        end

        function [F_b, M_b, F_c, M_c] = get_forces_and_moments(obj, state, disturbances)

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
           r_bc = obj.fin_params.translation;
           v_c = R_bc*(v_b + cross(w_b, r_bc));

           % Aerodynamic coefficients
           v_a = sqrt(v_c(1)^2+v_c(2)^2+v_c(3)^2);
           a = 343; % Speed of sound approximation (TODO: Replace with better approximation)
           M = v_a/a;
           AoA = atan2d(v_c(3),v_c(1));
           
           pos = state(2:4);
           altitude = pos(3);  % z componet of position
           [ca, cm, cn] = obj.aero_interp(M, AoA, altitude);

           S = obj.aero.sref;
           c = obj.aero.cbar;
           q = v_a * S;

           % Forces + Moments in component frame
           N = cn*q*S;
           A = ca*q*S;
           M = cm*q*c;
           F_c = [N; 0; A];
		   M_c = [0, M, 0];

           % Converting back to body frame
           R_cb = R_bc';
           F_b = R_cb*F_c;
           M_b = R_cb*M_c;
        end

        function [ca, cm, cn] = aero_interp(obj, mach, AoA_deg, alt)

            ca_F = griddedInterpolant({obj.aero.alpha(:), obj.aero.mach(:), obj.aero.alt(:)}, ...
                        obj.aero.ca, "linear");
            ca = ca_F(AoA_deg, mach, alt);

            cm_F = griddedInterpolant({obj.aero.alpha(:), obj.aero.mach(:), obj.aero.alt(:)}, ...
                        obj.aero.cm, "linear");
            cm = cm_F(AoA_deg, mach, alt);
            
            cn_F = griddedInterpolant({obj.aero.alpha(:), obj.aero.mach(:), obj.aero.alt(:)}, ...
                        obj.aero.cn, "linear");
			cn = cn_F(AoA_deg, mach, alt);
        end
	end
end