classdef RocketDynamics < handle
    properties
		body
		rocketFins
		motorParams 
		controller
        navigation
        aerodynamics = []
        
        thrustCurve
        totalImpulse
        burnTime
        massWet (1,1) double {mustBePositive} = 1
        massDry (1,1) double {mustBePositive} = 1
        moiWet (3,3) double = eye(3)
        moiDry (3,3) double = eye(3)
        cgWet (1,1) double {mustBePositive} = 1
        cgDry (1,1) double {mustBePositive} = 1

        useThrust = true
	end

	methods
        function obj = RocketDynamics(body, rocketFins, rocketParams, motorParams, controller, navigation, aerodynamics)
            arguments
                body
                rocketFins
                rocketParams
                motorParams
                controller
                navigation
                aerodynamics = []
            end

            obj.body = body;
            obj.rocketFins = rocketFins;
            obj.motorParams = motorParams;
            obj.thrustCurve = motorParams.tc;
            obj.totalImpulse = motorParams.totalImpulse;
            obj.burnTime = motorParams.burnTime;
            obj.massWet = rocketParams.massWet;
            obj.massDry = rocketParams.massDry;
            obj.moiWet = rocketParams.moiWet;
            obj.moiDry = rocketParams.moiDry;
            obj.cgWet = rocketParams.cgWet;
            obj.cgDry = rocketParams.cgDry;
            obj.controller = controller;
            obj.navigation = navigation;
            obj.aerodynamics = aerodynamics;
        end

        function currentThrust = getThrust(obj, t)
            % Error check
            if ~isnumeric(t)
                error('Error: Time must be numeric.')
            end
        
            % If beyond burn time, thrust is zero
            if ~obj.useThrust
                currentThrust = 0;
                return
            elseif t > obj.burnTime
                currentThrust = 0;
                return
            end
        
            % Extract time and thrust columns
            tc = obj.thrustCurve;
            time = tc(:, 1);
            thrust = tc(:, 2);
        
            % Linear interpolation
            currentThrust = interp1(time, thrust, t, 'linear', 0);
        end

        function [currentCG, currentMOI] = getCgMoi(obj, mass)
            % Error check
            if ~isnumeric(mass)
                error('Error: Mass must be numeric.')
            end

            % If beyond burn time, thrust is zero
            if mass <= obj.massDry
                currentCG = obj.cgDry;
                currentMOI = obj.moiDry;
                return
            end
            
            % Linear interpolation wrt mass
            currentCG = obj.cgDry + (mass - obj.massDry)/(obj.massWet - obj.massDry) .* (obj.cgWet - obj.cgDry);
            currentMOI = obj.moiDry + (mass - obj.massDry)/(obj.massWet - obj.massDry) .* (obj.moiWet - obj.moiDry);
        end

        function mDot = getMDot(obj, time)
            T = obj.getThrust(time);
            mDot = - T * (obj.massWet - obj.massDry) / obj.totalImpulse;
        end

        function [fB, mB] = getForcesAndMoments(obj, state, disturbances, time, xdot, breakPoints)
            
            % computes aero forces and moments
            if isempty(obj.aerodynamics)
                 [fBFins, mBFins] = obj.rocketFins.getForcesAndMoments(state, disturbances);
                 [fBBody, mBBody] = obj.body.getForcesAndMoments(state, disturbances);
                 fBAero = fBFins + fBBody;
                 mBAero = mBFins + mBBody;
             else
                 %[fBAero, mBAero] = obj.aerodynamics.getForcesAndMoments(state, disturbances);
                 fBAero = [0;0;0];
                 mBAero = [0;0;0];
             end

            fTB = [obj.getThrust(time); 0; 0];


            %TODO: sensorData = sensor.getSensorData(time, state, disturbances, CG)

            stateEstimate = obj.navigation.getStateEstimate(state, xdot, time);
            %[fBController, mBController] = obj.controller.getForcesAndMoments(state, obj, CG, breakPoints, time);

            fB = fTB + fBAero;
            mB = mBAero;
        end

        function compute_buildup(obj)
            obj.body.compute_buildup()
            obj.rocketFins.compute_buildup()
        end

		function draw(obj)
            % ========== ROCKET ==========
            r_body = obj.body.body_params;
            fins = obj.rocketFins.fin_params;
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



% DEPRECATED (sorry Kandarp)
% function [m, CG, moi] = interp_mass_CG_MOI(t)
%     % Need to pass in thrust curve somehow
%     tc = obj.motor_params.tc;
%     totalThrust = trapz(tc(1:2:end-1),tc(2:2:end)); 
%     if t < burnTime
%         % Proportionality constants
%         k_mass = (obj.mass_wet-obj.mass_dry)/totalThrust;
%         k_CG = (obj.CG_wet-obj.CG_dry)/totalThrust;
%         k_moi = (obj.moi_wet-obj.moi_dry)/totalThrust;
%         % Time + cumulative thrust calculations
%         curr_t = tc(tc(1:2:end-1) < t);
%         t_stop = curr_t(end); % Last tc point included in calculation
%         t_last = tc(find(tc == t_stop));
%         T_slope = (tc(t_last+3)-tc(t_last+1))/(tc(t_last+2)-tc(t_last));
%         cum_T = trapz(tc(1:2:t_last),tc(2:2:t_last+1)) + T_slope*(tc(t_last+2)-t_last);
%         % Thrust-based interpolation
%         m = obj.mass_wet - k_mass * cum_T;
%         CG = obj.CG_wet - k_CG * cum_T;
%         moi = obj.moi_wet - k_moi * cum_T;
%     else
%         m = obj.mass_dry;
%         CG = obj.CG_dry;
%         moi = obj.moi_dry;
%     end
% end