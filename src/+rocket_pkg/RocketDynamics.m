classdef RocketDynamics < handle
	properties
		body
		rocket_fins
		motor_params
		controller
        navigation

        mass_wet (1,1) double {mustBePositive} = 1
        mass_dry (1,1) double {mustBePositive} = 1
        moi_wet (3,3) double = eye(3)
        moi_dry (3,3) double = eye(3)
        CG_wet (1,1) double {mustBePositive} = 1
        CG_dry (1,1) double {mustBePositive} = 1
	end

	methods
        function obj = RocketDynamics(body, rocket_fins, rocket_params, motor_params)
            obj.body = body;
            obj.rocket_fins = rocket_fins;
            obj.motor_params = motor_params;
            obj.mass_wet = rocket_params.mass_wet;
            obj.mass_dry = rocket_params.mass_dry;
            obj.moi_wet = rocket_params.moi_wet;
            obj.moi_dry = rocket_params.moi_dry;
            obj.CG_wet = rocket_params.CG_wet;
            obj.CG_dry = rocket_params.CG_dry;
        end

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
            [F_b_fins, M_b_fins] = obj.rocket_fins.get_forces_and_moments(state, disturbances);
            [F_b_body, M_b_body] = obj.body.get_forces_and_moments(state, disturbances);
            
            %TODO: sensor_data = sensor.get_sensor_data(time, state, disturbances, CG)
            %TODO: state_estimate = navigation.get_state_estimate(state, disturbances)
            %[F_b_controller, M_b_controller] = obj.controller.get_forces_and_moments(state, obj, CG, breakPoints, time);
            F_b = F_b_fins + F_b_body;
            M_b = M_b_fins + M_b_body;
			%F_b = F_b_fins;
			%M_b = M_b_fins;
        end

        function compute_buildup(obj)
            obj.body.compute_buildup()
            obj.rocket_fins.compute_buildup()
        end

		function draw(obj)
			% TODO: draw rocket (rough draft below, need to get access to rocket size parameters)
            
        %     figure; hold on; axis equal
        %     xlabel('X'); ylabel('Y'); zlabel('Z');
		% 
		% 	D_body = obj.body.body_params.rocket_diameter;
		% 	%L_body = 
		% 
        %     % Body (cylinder)
        %     [Xb, Yb, Zb] = cylinder(D_body/2, 50);
        %     Zb = Zb * L_body;
        %     surf(Xb, Yb, Zb, 'FaceColor', [0.8 0.8 0.9], 'EdgeColor', 'none');
		% 
        %     % Nose (cone)
        %     [Xn, Yn, Zn] = cylinder([D_body/2 0], 50);
        %     Zn = Zn * L_nose + L_body;
        %     surf(Xn, Yn, Zn, 'FaceColor', [0.9 0.7 0.7], 'EdgeColor', 'none');
		% 
        %     % Fins
        %     theta = linspace(0, 2*pi, n_fins+1);
        %     for i = 1:n_fins
        %         % Fin coordinates in XY-plane
        %         finX = [L_body-0.1*L_body, L_body-0.1*L_body, L_body-L_fin];
        %         finY = [D_body/2, D_body/2+H_fin, D_body/2];
        %         finZ = [0, 0, 0];
		% 
        %         % Rotate fin around Z-axis
        %         R = [cos(theta(i)) -sin(theta(i)); sin(theta(i)) cos(theta(i))];
        %         finCoords = R * [finY; finZ];
		% 
        %         % Draw fin (patch)
        %         patch(finX, finCoords(1,:), finCoords(2,:), [0.6 0.6 0.6]);
        %     end
        %     camlight; lighting gouraud
		end

    end
end