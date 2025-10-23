classdef RocketDynamics < handle&Exportable

	properties (SetAccess = private)
		body
		rocket_fins
		motor_params
		controller
        navigation
        aerodynamics

        mass_wet (1,1) double {mustBePositive} = 1
        mass_dry (1,1) double {mustBePositive} = 1
        moi_wet (3,3) double = eye(3)
        moi_dry (3,3) double = eye(3)
        CG_wet (1,1) double {mustBePositive} = 1
        CG_dry (1,1) double {mustBePositive} = 1
	end

	methods
        function obj = RocketDynamics(varargin)
            if nargin == 0
                % Empty constructor for Exportable (loading from MAT)
                return

            elseif nargin == 4
                body = varargin{1};
                rocket_fins = varargin{2};
                rocket_params = varargin{3};
                motor_params = varargin{4};

                obj.body = body;
                obj.rocket_fins = rocket_fins;
                obj.motor_params = motor_params;
                obj.mass_wet = rocket_params.mass_wet;
                obj.mass_dry = rocket_params.mass_dry;
                obj.moi_wet = rocket_params.moi_wet;
                obj.moi_dry = rocket_params.moi_dry;
                obj.CG_wet = rocket_params.CG_wet;
                obj.CG_dry = rocket_params.CG_dry;
            else
                error('RocketDynamics constructor expects either 0 or 4 input arguments.');
            end
        end % wrap added by Zhanna S

        function restoreFromStruct(obj, s)
        % Restore all properties from a struct (bypasses SetAccess=private)
            f = fieldnames(s); % Get all field names from the struct 's'
            for i = 1:numel(f) % Loop thru each field
                try
                    obj.(f{i}) = s.(f{i}); % Assign struct to the object property
                catch ME % Warning if fails so it doesn't crash
                    warning('Could not restore %s: %s', f{i}, ME.message);
                end
            end
        end % function added by Zhanna S

        function [m, CG, moi] = interp_mass_CG_MOI(t)
            % Need to pass in thrust curve somehow
            tc = obj.motor_params.tc;
            totalThrust = trapz(tc(1:2:end-1),tc(2:2:end)); 
            if t < burnTime
                % Proportionality constants
                k_mass = (obj.mass_wet-obj.mass_dry)/totalThrust;
                k_CG = (obj.CG_wet-obj.CG_dry)/totalThrust;
                k_moi = (obj.moi_wet-obj.moi_dry)/totalThrust;
                % Time + cumulative thrust calculations
                curr_t = tc(tc(1:2:end-1) < t);
                t_stop = curr_t(end); % Last tc point included in calculation
                t_last = tc(find(tc == t_stop));
                T_slope = (tc(t_last+3)-tc(t_last+1))/(tc(t_last+2)-tc(t_last));
                cum_T = trapz(tc(1:2:t_last),tc(2:2:t_last+1)) + T_slope*(tc(t_last+2)-t_last);
                % Thrust-based interpolation
                m = obj.mass_wet - k_mass * cum_T;
                CG = obj.CG_wet - k_CG * cum_T;
                moi = obj.moi_wet - k_moi * cum_T;
            else
                m = obj.mass_dry;
                CG = obj.CG_dry;
                moi = obj.moi_dry;
            end
        end

        function [F_b, M_b] = get_forces_and_moments(obj, state, disturbances, time, breakPoints)
            
            % computes aero forces and moments 
            % Need to think on how to implement wind frame
            if isempty(obj.aerodynamics)
                [F_b_fins, M_b_fins] = obj.rocket_fins.get_forces_and_moments(state, disturbances);
                [F_b_body, M_b_body] = obj.body.get_forces_and_moments(state, disturbances);
                F_b_aero = F_b_fins + F_b_body;
                M_b_aero = M_b_fins + M_b_body;
            else
                [F_b_aero, M_b_aero] = obj.aerodynamics
            end
            
            %TODO: sensor_data = sensor.get_sensor_data(time, state, disturbances, CG)
            %TODO: state_estimate = navigation.get_state_estimate(state, disturbances)
            %[F_b_controller, M_b_controller] = obj.controller.get_forces_and_moments(state, obj, CG, breakPoints, time);
            F_b = F_b_aero;
            M_b = M_b_aero;
			%F_b = F_b_fins;
			%M_b = M_b_fins;
        end

        function compute_buildup(obj)
            obj.body.compute_buildup()
            obj.rocket_fins.compute_buildup()
        end

		function draw(obj)
            % ========== ROCKET ==========
            r_body = obj.body.body_params;
            fins = obj.rocket_fins.fin_params;
            r = r_body.rocket_diameter/2;
            x_nose = linspace(r_body.nose_cone_length, 0, 100);
            
            % Tangent ogive nose cone profile
            y_profile = obj.body.get_nose_radii(x_nose); 
            
            % Cylindrical body
            x_body = linspace(r_body.nose_cone_length, r_body.nose_cone_length + r_body.body_length, 50);
            y_body = r * ones(size(x_body));
            
            % Combine profile
            x = [x_nose, x_body];
            y = [y_profile, y_body];
            
            % Revolve a 2D curve (x,y) about the x-axis to form a surface
            theta = linspace(0, 2*pi, 120);
            X = repmat(x(:), 1, 120);
            Y = repmat(y(:), 1, 120).*cos(theta);
            Z = repmat(y(:), 1, 120).*sin(theta);
            
            % Plot body
            figure; hold on;
            surf(X, Y, Z, 'FaceColor', [0.8 0.8 0.9], 'EdgeColor', 'none');
            axis equal; grid on;
            xlabel('Length (m)'); ylabel('Y (m)'); zlabel('Z (m)');
            title('3D Rocket with Fins');
            camlight headlight; lighting gouraud;
            
            % ========== FINS ==========
            % Fin base position (at end of body)
            x_base = r_body.nose_cone_length + r_body.body_length - fins.root_chord;  % where root leading edge starts
            finColor = [0.2 0.5 0.8];
            
            % Define one fin shape in X–Y plane before rotation
            LE_root = [0, 0];
            TE_root = [fins.root_chord, 0];
            LE_tip  = [fins.fin_span * tand(fins.sweep_deg), fins.fin_span];
            TE_tip  = [LE_tip(1) + fins.tip_chord, fins.fin_span];
            
            finX = [LE_root(1), TE_root(1), TE_tip(1), LE_tip(1)];
            finY = [LE_root(2), TE_root(2), TE_tip(2), LE_tip(2)];
            
            % For each fin, rotate and position around body
            for i = 1:fins.num_fins
                theta2 = 2*pi*(i-1)/fins.num_fins;  % angular position
                % Rotation matrix around X-axis (rocket axis)
                Rz = [1 0 0; 0 cos(theta2) -sin(theta2); 0 sin(theta2) cos(theta2)];

                % Shift fin so its root attaches to rocket body
                Xf = finX + x_base;
                Yf = r + finY;
                Zf = zeros(size(finY));
                
                % Stack to rotate (not necessary here because we rotate by construction)
                fin_local = [Xf; Yf; Zf];
                fin = Rz * fin_local;
                
                % Draw fin as patch
                fill3(Xf, fin(2,:), fin(3,:), finColor, 'FaceAlpha', 0.9, 'EdgeColor', 'k');
            end
            
            view(30,20);
        end
    end
end