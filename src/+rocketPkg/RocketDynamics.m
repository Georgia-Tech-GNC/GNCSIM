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
            obj.thrustCurve = motorParams.thrustCurve;
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

        function [currentCg, currentMoi] = getCgMoi(obj, mass)
            % Error check
            if ~isnumeric(mass)
                error('Error: Mass must be numeric.')
            end

            % If beyond burn time, thrust is zero
            if mass <= obj.massDry
                currentCg = obj.cgDry;
                currentMoi = obj.moiDry;
                return
            end

            % Linear interpolation wrt mass
            currentCg = obj.cgDry + (mass - obj.massDry)/(obj.massWet - obj.massDry) .* (obj.cgWet - obj.cgDry);
            currentMoi = obj.moiDry + (mass - obj.massDry)/(obj.massWet - obj.massDry) .* (obj.moiWet - obj.moiDry);
        end

        function mDot = getMDot(obj, time)
            T = obj.getThrust(time);
            mDot = - T * (obj.massWet - obj.massDry) / obj.totalImpulse;
        end

        function [fB, mB] = getForcesMoments(obj, state, disturbances, time, xdot, breakPoints)

            % computes aero forces and moments
            if isempty(obj.aerodynamics)
                 [fBFins, mBFins] = obj.rocketFins.getForcesMoments(state, disturbances);
                 [fBBody, mBBody] = obj.body.getForcesMoments(state, disturbances);
                 fBAero = fBFins + fBBody;
                 mBAero = mBFins + mBBody;
             else
                 %[fBAero, mBAero] = obj.aerodynamics.getForcesMoments(state, disturbances);
                 fBAero = [0;0;0];
                 mBAero = [0;0;0];
             end

            fTB = [obj.getThrust(time); 0; 0];
            
            %TODO: sensorData = sensor.getSensorData(time, state, disturbances, CG)

            %TODO: %stateEstimate = obj.navigation.getStateEstimate(state, xdot, time);
            %TODO: %[fBController, mBController] = obj.controller.getForcesMoments(state, obj, CG, breakPoints, time);

            fB = fTB + fBAero;
            mB = mBAero;
        end

        function computeBuildup(obj)
            obj.body.computeBuildup()
            obj.rocketFins.computeBuildup()
        end

		function draw(obj)
            % ========== ROCKET ==========
            rBody = obj.body.bodyParams;
            fins = obj.rocketFins.finParams;
            r = rBody.rocketDiameter/2;
            xNose = linspace(rBody.noseLength, 0, 100);

            % Tangent ogive nose cone profile
            yProfile = obj.body.getNoseRadii(xNose);

            % Cylindrical body
            xBody = linspace(rBody.noseLength, rBody.noseLength + rBody.bodyLength, 50);
            yBody = r * ones(size(xBody));

            % Combine profile
            x = [xNose, xBody];
            y = [yProfile, yBody];

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
            xBase = rBody.noseLength + rBody.bodyLength - fins.rootChord;  % where root leading edge starts
            finColor = [0.2 0.5 0.8];

            % Define one fin shape in X–Y plane before rotation
            leRoot = [0, 0];
            teRoot = [fins.rootChord, 0];
            leTip  = [fins.finSpan * tand(fins.sweepDeg), fins.finSpan];
            teTip  = [leTip(1) + fins.tipChord, fins.finSpan];

            finX = [leRoot(1), teRoot(1), teTip(1), leTip(1)];
            finY = [leRoot(2), teRoot(2), teTip(2), leTip(2)];

            % For each fin, rotate and position around body
            for i = 1:fins.numFins
                theta2 = 2*pi*(i-1)/fins.numFins;  % angular position
                % Rotation matrix around X-axis (rocket axis)
                Rz = [1 0 0; 0 cos(theta2) -sin(theta2); 0 sin(theta2) cos(theta2)];

                % Shift fin so its root attaches to rocket body
                Xf = finX + xBase;
                Yf = r + finY;
                Zf = zeros(size(finY));

                % Stack to rotate (not necessary here because we rotate by construction)
                finLocal = [Xf; Yf; Zf];
                fin = Rz * finLocal;

                % Draw fin as patch
                fill3(Xf, fin(2,:), fin(3,:), finColor, 'FaceAlpha', 0.9, 'EdgeColor', 'k');
            end

            view(30,20);
        end
    end
end



% DEPRECATED (sorry Kandarp)
% function [m, cg, moi] = interpMassCgMoi(t)
%     % Need to pass in thrust curve somehow
%     tc = obj.motorParams.tc;
%     totalThrust = trapz(tc(1:2:end-1),tc(2:2:end));
%     if t < burnTime
%         % Proportionality constants
%         kMass = (obj.massWet-obj.massDry)/totalThrust;
%         kCg = (obj.cgWet-obj.cgDry)/totalThrust;
%         kMoi = (obj.moiWet-obj.moiDry)/totalThrust;
%         % Time + cumulative thrust calculations
%         currT = tc(tc(1:2:end-1) < t);
%         tStop = currT(end); % Last tc point included in calculation
%         tLast = tc(find(tc == tStop));
%         tSlope = (tc(tLast+3)-tc(tLast+1))/(tc(tLast+2)-tc(tLast));
%         cumT = trapz(tc(1:2:tLast),tc(2:2:tLast+1)) + tSlope*(tc(tLast+2)-tLast);
%         % Thrust-based interpolation
%         m = obj.massWet - kMass * cumT;
%         cg = obj.cgWet - kCg * cumT;
%         moi = obj.moiWet - kMoi * cumT;
%     else
%         m = obj.massDry;
%         cg = obj.cgDry;
%         moi = obj.moiDry;
%     end
% end