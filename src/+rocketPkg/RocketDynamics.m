% Main Authors: Karsten Caillet (kcaillet3@gatech.edu)
%               David Reynolds (dreynolds45@gatech.edu)
%               Kandarp Vadia (kvadia3@gatech.edu)
%               Aditya Swami (aswami8@gatech.edu)

classdef RocketDynamics < handle

    properties (Constant)
        idx = StateIndices
    end

    properties
		body
		rocketFins
		motorParams
        activeParachute         % Currently active parachute (linked list)
        thrustCurve
        totalImpulse
        burnTime
        massWet 
        massDry 
        moiWet  
        moiDry  
        cgWet 
        cgDry 

        actuatorModel

        aerodynamicsCFD = []
	end

	methods
        function obj = RocketDynamics(body, rocketFins, rocketParams, motorParams, actuatorModel, firstParachute, aerodynamicsCFD)
            arguments
                body                % DATCOM
                rocketFins          % DATCOM
                rocketParams
                motorParams
                actuatorModel
                firstParachute      % Head of parachute linked list
                aerodynamicsCFD = []   % defaults to empty
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

            obj.actuatorModel = actuatorModel;

            obj.aerodynamicsCFD = aerodynamicsCFD;

            obj.activeParachute = firstParachute;
        end

        function currentThrust = getThrust(obj, t)
            % Error check
            if ~isnumeric(t)
                error('Error: Time must be numeric.')
            end
        
            % If beyond burn time, thrust is zero
            if t > obj.burnTime
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

        function xDot = rocketModelTrajOptWrapper(obj, state, controlInput, disturbances, time, flightState, log, useGravity, useThrust)
            arguments
                obj
                state
                controlInput
                disturbances % wind
                time  % current time in seconds
                flightState = flightPkg.FlightState.POWERED_ASCENT
                log = []  % Optional, defaults to empty
                useGravity = true
                useThrust = true
            end

            [Nx, Nt] = size(state);
            xDot = zeros(Nx, Nt);

            for i = 1:Nt
                xDot(:,i) = obj.rocketModelControlWrapper(state(:,i), controlInput(:,i), disturbances, time(i), flightState, log, useGravity, useThrust);
            end
        end

        function xDot = rocketModelControlWrapper(obj, state, controlInput, disturbances, time, flightState, log, useGravity, useThrust)
            arguments
                obj
                state
                controlInput
                disturbances % wind
                time  % current time in seconds
                flightState = flightPkg.FlightState.POWERED_ASCENT
                log = []  % Optional, defaults to empty
                useGravity = true
                useThrust = true
            end
            actuatorState = obj.actuatorModel.allocationMatrix * controlInput;
            xDot = obj.rocketModel(state, actuatorState, disturbances, time, flightState, log, useGravity, useThrust);
        end

		function xDot = rocketModel(obj, state, actuatorState, disturbances, time, flightState, log, useGravity, useThrust)
            arguments
                obj
                state
                actuatorState
                disturbances % wind
                time  % current time in seconds
                flightState = flightPkg.FlightState.POWERED_ASCENT
                log = []  % Optional, defaults to empty
                useGravity = true
                useThrust = true
            end

            [fB, mB] = obj.getForcesMoments(state, actuatorState, disturbances, time, flightState, log, useGravity, useThrust);
            xDot = obj.flatEarthEOM(state, time, fB, mB);
        end

        function [fB, mB] = getForcesMoments(obj, state, actuatorState, disturbances, time, flightState, log, useGravity, useThrust)
            arguments
                obj
                state 
                actuatorState  
                disturbances  
                time % current time in seconds
                flightState
                log = []  % Optional, defaults to empty
                useGravity = true
                useThrust = true
            end

            import flightPkg.FlightState

            % Extract common state variables
            mass = state(obj.idx.mass);
            rocketQuat = state(obj.idx.quat);
            cBI = Utils.dirCosine(rocketQuat);
            [currentCg, ~] = obj.getCgMoi(mass);

            % Nested helper functions
            function fGBody = computeGravity()
                fGBody = cBI * [0; 0; -9.81 * useGravity * mass];
            end

            function fTB = computeThrust()
                fTB = [obj.getThrust(time) * useThrust; 0; 0];
            end

            function [fBAero, mBAero] = computeAero()
                if isempty(obj.aerodynamicsCFD)
                    [fBFins, mBFins] = obj.rocketFins.getForcesMoments(state, [], currentCg, disturbances);
                    [fBBody, mBBody] = obj.body.getForcesMoments(state, disturbances);
                    fBAero = fBFins + fBBody;
                    mBAero = mBFins + mBBody;
                else
                    [fBAero, mBAero] = obj.aerodynamicsCFD.getForcesMoments(state, disturbances);
                end
            end

            function [fBActuator, mBActuator] = computeActuator()
                [fBActuator, mBActuator] = obj.actuatorModel.getForcesMoments(state, actuatorState, currentCg, disturbances);
            end

            function fBParachute = computeParachute()
                rocketAlt = state(obj.idx.z);
                rocketVel = state(obj.idx.vel);

                if isempty(obj.activeParachute)
                    fBParachute = [0; 0; 0];
                    return;
                end

                % Check if we should transition to next parachute
                if obj.activeParachute.shouldDeployNext(rocketAlt)
                    obj.activeParachute = obj.activeParachute.getNext();
                end

                fBParachute = obj.activeParachute.getForcesAndMoments(rocketVel, rocketQuat, disturbances);
            end

            % Assemble forces and moments based on flight state

            % TODO: This is somewhat hardcoded for fin tabs, actuator model for
            % lorax will wrap thrust, but that means actuator model should
            % only be called during powered ascent - need to find a
            % solution
            switch flightState
                case FlightState.PRELAUNCH
                    fB = [0; 0; 0];
                    mB = [0; 0; 0];

                case FlightState.LAUNCH_RAIL
                    fGBody = computeGravity();
                    fTB = computeThrust();
                    [fBAero, ~] = computeAero();
                    [fBActuator, ~] = computeActuator();

                    fTotal = fGBody + fTB + fBAero + fBActuator;
                    fB = [fTotal(1); 0; 0];
                    mB = [0; 0; 0];

                    if ~isempty(log)
                        log.add(time, 'aeroforces', [fBAero(1); 0; 0]);
                        log.add(time, 'thrust', obj.getThrust(time));
                    end

                case FlightState.POWERED_ASCENT
                    fGBody = computeGravity();
                    fTB = computeThrust();
                    [fBAero, mBAero] = computeAero();
                    [fBActuator, mBActuator] = computeActuator();

                    fB = fGBody + fTB + fBAero + fBActuator;
                    mB = mBAero + mBActuator;

                    if ~isempty(log)
                        log.add(time, 'aeroforces', fBAero);
                        log.add(time, 'aeromoments', mBAero);
                        log.add(time, 'thrust', obj.getThrust(time));
                    end

                case FlightState.COASTING_ASCENT
                    fGBody = computeGravity();
                    [fBAero, mBAero] = computeAero();
                    [fBActuator, mBActuator] = computeActuator();

                    fB = fGBody + fBAero + fBActuator;
                    mB = mBAero + mBActuator;

                    if ~isempty(log)
                        log.add(time, 'aeroforces', fBAero);
                        log.add(time, 'aeromoments', mBAero);
                    end

                case FlightState.DESCENT
                    fGBody = computeGravity();
                    fBParachute = computeParachute();

                    fB = fGBody + fBParachute;
                    mB = [0; 0; 0];

                    if ~isempty(log)
                        log.add(time, 'parachuteforces', fBParachute);
                    end

                case FlightState.LANDED
                    fB = [0; 0; 0];
                    mB = [0; 0; 0];

                otherwise
                    error('Unknown flight state: %s', string(flightState));
            end
        end
        
        function stateDot = flatEarthEOM(obj, state, time, fB, mB)
			arguments
                obj
                state 
                time
				fB
				mB
            end
            mass = state(obj.idx.mass);
            vel = state(obj.idx.vel);
            quat = state(obj.idx.quat);
            omega = state(obj.idx.omega);

            % preventing mass to go below massDry
            mass = max(mass, obj.massDry);

            [~, Ib] = obj.getCgMoi(mass);

            cBI = Utils.dirCosine(quat);
            cIB = cBI.';

            massDot = obj.getMDot(time);
            posDot = cIB * vel;
            velDot = fB / mass - cross(omega, vel);
            quatDot = Utils.quatDerivative(quat, omega);
            omegaDot = Ib\(mB - cross(omega, Ib*omega));
            stateDot = [massDot; posDot; velDot; quatDot; omegaDot];
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
    end
end
