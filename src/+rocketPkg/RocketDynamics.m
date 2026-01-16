% Main Authors: Karsten Caillet (kcaillet3@gatech.edu)
%               David Reynolds (dreynolds45@gatech.edu)
%               Kandarp Vadia (kvadia3@gatech.edu)
%               Aditya Swami (aswami8@gatech.edu)

classdef RocketDynamics < handle
    properties
		body
		rocketFins
		motorParams 
		controller
        aerodynamics = []
        firstChute
        secondChute
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
        allocationMatrix
        
        uncertainMassWet = []
        uncertainMassDry = []
        uncertainMoiWet = []
        uncertainMoiDry = []
        uncertainCgWet = []
        uncertainCgDry = []
	end

	methods
        function obj = RocketDynamics(body, rocketFins, rocketParams, motorParams, controller, parachuteObjects, aerodynamics)
            arguments
                body
                rocketFins
                rocketParams
                motorParams
                controller
                parachuteObjects
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

            obj.moiWet = diag([rocketParams.rotationalMoiWet, rocketParams.longitudinalMoiWet, rocketParams.longitudinalMoiWet]);
            obj.moiDry = diag([rocketParams.rotationalMoiDry, rocketParams.longitudinalMoiDry, rocketParams.longitudinalMoiDry]);

            obj.cgWet = rocketParams.cgXWet;
            obj.cgDry = rocketParams.cgXDry;

            obj.controller = controller;
            obj.aerodynamics = aerodynamics;
            
            % This is a mapping from pitch, yaw and roll to the four fin
            % deflections
            obj.allocationMatrix = [-1, 0, 1, 0;
                                    0, 1, 0, -1;
                                    0, -1, 0, -1].';

        end
        
        function setUncertainParameters(obj, massWet, massDry, moiWet, moiDry, cgWet, cgDry)
            obj.uncertainMassWet = massWet;
            obj.uncertainMassDry = massDry;
            obj.uncertainMoiWet = moiWet;
            obj.uncertainMoiDry = moiDry;
            obj.uncertainCgWet = cgWet;
            obj.uncertainCgDry = cgDry;
        end
        
        function clearUncertainParameters(obj)
            obj.uncertainMassWet = [];
            obj.uncertainMassDry = [];
            obj.uncertainMoiWet = [];
            obj.uncertainMoiDry = [];
            obj.uncertainCgWet = [];
            obj.uncertainCgDry = [];
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
            if ~isnumeric(mass) && ~isa(mass, 'ureal')
                error('Error: Mass must be numeric or ureal.')
            end

            masswet = obj.uncertainMassWet;
            massdry = obj.uncertainMassDry;
            moiwet = obj.uncertainMoiWet;
            moidry = obj.uncertainMoiDry;
            cgwet = obj.uncertainCgWet;
            cgdry = obj.uncertainCgDry;
            
            % In case we are not computing gains, which is most of the time
            if isempty(masswet)
                masswet = obj.massWet;
                massdry = obj.massDry;
                moiwet = obj.moiWet;
                moidry = obj.moiDry;
                cgwet = obj.cgWet;
                cgdry = obj.cgDry;
            end

            if mass <= massdry
                currentCg = cgdry;
                currentMoi = moidry;
                return
            end

            currentCg = cgdry + (mass - massdry)/(masswet - massdry) .* (cgwet - cgdry);
            currentMoi = moidry + (mass - massdry)/(masswet - massdry) .* (moiwet - moidry);
        end

        function mDot = getMDot(obj, time)
            
            masswet = obj.uncertainMassWet;
            massdry = obj.uncertainMassDry;
            
            % In case we are not computing gains, which is most of the time
            if isempty(masswet)
                masswet = obj.massWet;
                massdry = obj.massDry;
            end

            T = obj.getThrust(time);
            mDot = - T * (masswet - massdry) / obj.totalImpulse;
        end

        function [fB, mB] = getForcesMoments(obj, state, deflections, disturbances, time, log)
            % computes aero forces and moments
			[currentCg, ~] = getCgMoi(obj, state(1));

            % % Get Parachute Forces
            % rocketVel = state(2:4);
            % rocketAngularVel = state(12:14);
            % rocketQuat = state(8:11);
            % if atApogee(state, time)
            %     [F_rocket_com, M_rocket_com] = obj.firstChute.getForcesAndMoments(rocketVel, rocketAngularVel, rocketQuat, timeStep);
            % 
            % end


            if isempty(obj.aerodynamics)
                 [fBFins, mBFins] = obj.rocketFins.getForcesMoments(state, deflections, currentCg, disturbances);
                 [fBBody, mBBody] = obj.body.getForcesMoments(state, disturbances);
                 fBAero = fBFins + fBBody;
                 mBAero = mBFins + mBBody;
            else
                 [fBAero, mBAero] = obj.aerodynamics.getForcesMoments(state, disturbances);
            end

            fTB = [obj.getThrust(time); 0; 0];
            %[fBController, mBController] = obj.controller.getForcesMoments(stateEstimate, obj, CG, breakPoints, time, log);
            
            

            if ~isempty(log)
                log.add(time, 'aeroforces', fBAero);
                log.add(time, 'aeromoments', mBAero);
                
                log.add(time, 'thrust', obj.getThrust(time));
                %log.add(time, 'controlmoments', mBController);
            end

            fB = fTB + fBAero;
            mB = mBAero;
        end

        function computeBuildup(obj)
            % Main Author: David Reynolds
            obj.body.computeBuildup()
            obj.rocketFins.computeBuildup()
			
			% Added scale factors in order to line up with open rocket
			obj.body.scaleFactor = 1.51;
			obj.body.momentScaleFactor = 0.21;
			[~, numFins] = size(obj.rocketFins.fins);
			for ii=1:numFins
				obj.rocketFins.fins(ii).scaleFactor = 1.97;
			end
        end

		function draw(obj)
            % Main Author: Kandarp Vadia
            % ========== ROCKET ==========
            rBody = obj.body.bodyParams;
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
            for ii=1:length(obj.rocketFins.finParamsArr)
                fins = obj.rocketFins.finParamsArr(ii);
                translation = fins.translation;

                %xBase = rBody.noseLength + rBody.bodyLength - fins.rootChord;  % where root leading edge starts
                xBase = translation(1);
                finColor = [0.2 0.5 0.8];
    
                % Define one fin shape in X–Y plane before rotation
                leRoot = [0, 0];
                teRoot = [fins.rootChord, 0];
                leTip  = [fins.finSpan * tand(fins.sweepDeg), fins.finSpan];
                teTip  = [leTip(1) + fins.tipChord, fins.finSpan];
    
                finX = [leRoot(1), teRoot(1), teTip(1), leTip(1)];
                finY = [leRoot(2), teRoot(2), teTip(2), leTip(2)];
    
                % For each fin, rotate and position around body
                for jj = 1:fins.numFins
                    theta2 = 2*pi*(jj-1)/fins.numFins;  % angular position
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
            end

            view(30,20);
        end

        function [reachedApogee] = atApogee(state, time)
            persistent timeCheck
            
            velBody = state(5:7);
            body2InertialQuat = state(8:11);
            body2InertialDCM = utils.dirCosine(body2InertialQuat);
            velInertial = body2InertialDCM * velBody;
            velNorth = velInertial(3);
            
            if velNorth < 0
                if isempty(timeCheck)
                    timeCheck = time + 1;
                elseif time >= timeCheck
                    reachedApogee = true;
                else
                    reachedApogee = false;
                end
            else
                reachedApogee = false;
            end
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