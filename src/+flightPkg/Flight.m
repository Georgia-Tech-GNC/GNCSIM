% Main Authors: David Reynolds (dreynolds45@gatech.edu)
%               Karsten Caillet (kcaillet3@gatech.edu)

classdef Flight < handle
    properties
        rocketDynamics
        environment
        breakPoints
        navigation
        flightParams

        totalTime
        currentTime
        dt

        rocketInitialState

	end
	
    methods
        function obj = Flight(rocketDynamics, environment, breakPoints, navigation, flightParams)
            obj.rocketDynamics = rocketDynamics;
            obj.environment = environment;
            obj.breakPoints = breakPoints;
            obj.navigation = navigation;
            obj.flightParams = flightParams;

            obj.dt = obj.flightParams.dt;
            obj.rocketInitialState = obj.flightParams.x0;
            obj.totalTime = obj.flightParams.totalTime;
                
        end

        function log = runSim(obj, log)
            obj.currentTime = 0.0;
            rocketState = obj.rocketInitialState(:);
            %u = [0; -1; 0; -1];  % positive roll (correct)
			%u = [0; 1; 0; -1];  % positive yaw (correct)
            %u = [-1; 0; 1; 0];  % positive pitch (correct)
            u = [0; 0; 0];
            
            % Initialize logger time step
            log.initializeTimestep(obj.currentTime);
            
            % Get state estimate
            rocketStateEstimate = obj.navigation.getStateEstimate(rocketState, zeros(14,1), obj.currentTime, log);
            
            rocketStateDot = obj.rocketDynamics.rocketModel(rocketState, u, obj.currentTime, log); % thrust starts at t=0s

            log.add(obj.currentTime, 'states', rocketState);
            log.add(obj.currentTime, 'statedots', rocketStateDot);
			log.add(obj.currentTime, 'stateestimates', rocketStateEstimate);

            for ii = 1:log.maxSteps - 1

                obj.currentTime = ii * obj.dt;
                log.initializeTimestep(obj.currentTime);

                [K2, K3] = obj.getGains(rocketState);
                [pitchErr, yawErr, rollErr] = obj.getError(rocketState);
                
                pitchControl = K3 * pitchErr;
				yawControl = K3 * yawErr;
                rollControl = K2 * rollErr;

                rocketControlRad = [pitchControl, yawControl, 0].';
				rocketControl = rad2deg(rocketControlRad);
                
                rocketDyn = @(x) obj.rocketDynamics.rocketModel(x, rocketControl, obj.currentTime);
                rocketState = Utils.RK4(rocketDyn, rocketState, obj.dt);
                quat = rocketState(8:11);
                quatNorm = Utils.normalizeQuat(quat);
                rocketState(8:11) = quatNorm;  

                rocketStateDot = obj.rocketDynamics.rocketModel(rocketState, rocketControl, obj.currentTime, log);
                rocketStateEstimate = obj.navigation.getStateEstimate(rocketState, rocketStateDot, obj.currentTime, log);

                log.add(obj.currentTime, 'states', rocketState);
                log.add(obj.currentTime, 'statedots', rocketStateDot);
                log.add(obj.currentTime, 'stateestimates', rocketStateEstimate);

            end
        end

        function [pitchErr, yawErr, rollErr] = getError(obj, rocketState)
            time = obj.currentTime;
            x3Pitch = controllerPkg.pitchReduce(rocketState);
            x3Yaw = controllerPkg.pitchReduce(rocketState, true);
            x2Roll = controllerPkg.rollReduce(rocketState);
			
			if time < 5
				yawRef = deg2rad([0; 0; 5*time]);
			elseif time >= 5 && time < 10
				yawRef = deg2rad([0; 0; 25 - (time-5)*5]);
			else
				yawRef = deg2rad([0; 0; 0]);
			end
			
			%yawRef = deg2rad([0; 0; 0]);
            pitchRef = deg2rad([0; 0; 0]);
            rollRef = deg2rad([0; 5]);

            pitchErr = pitchRef - x3Pitch;
            yawErr = yawRef - x3Yaw;
            rollErr = rollRef - x2Roll;
            %{
            q0 = x14(8);
            q1 = x14(9);
            q2 = x14(10);
            q3 = x14(11);

            quat = quaternion([q0, q1, q2, q3]);  % 1 x 4 matrix
            quat_y90 = quaternion(Utils.eulerToQuat([0, 90, 0])');
            quat_rotated = quat * quat_y90;
            eul = quat2eul(quat_rotated, 'XYZ');
            %}
        end

        function [K2, K3] = getGains(obj, state)
            N = length(obj.breakPoints.breakPointArr);
            speedArr = zeros(1, N);
            for ii=1:length(obj.breakPoints.breakPointArr)
                params = obj.breakPoints.breakPointArr.breakParam;
                speedArr(ii) = params.speed;
            end
            currentSpeed = norm(state(5:7));
            [~,I] = min(abs(speedArr - currentSpeed));
            K2 = obj.breakPoints.breakPointArr(I).K2;
            K3 = obj.breakPoints.breakPointArr(I).K3;
        end
		
		%{
        function xDot = rocketModel(obj, state, control, log)
            arguments
                obj
                state 
                control  % array of 3 elements
                log = []  % Optional, defaults to empty
            end

            altitude = state(4);
            %disturbances.wind = obj.environment.getWindspeed(altitude);
            disturbances.wind = zeros(3);
            
            % Mapping from control deflections (pitch, yaw, roll) to fin
            % deflections (fin 1, 2, 3, 4)
            deflections = obj.rocketDynamics.allocationMatrix * control;

            [fB, mB] = obj.rocketDynamics.getForcesMoments(state, deflections, disturbances, obj.currentTime, log);
            xDot = obj.flatEarth_EOM(state, fB, mB);
        end
		%}
		
        
        
		%{
        function xDot = rocketModelDeflection(obj, state, control, log)
			arguments
                obj
                state 
				control
                log = []  % Optional, defaults to empty
            end

            altitude = state(4);
            disturbances.wind = zeros(3);
            obj.currentTime = 0;

            [fB, mB] = obj.rocketDynamics.getForcesMoments(state, control, disturbances, obj.currentTime, log);
            
            xDot = obj.flatEarth_EOM(state, fB, mB);

		    if ~isempty(log)
                log.add(obj.currentTime, 'statedots', xDot);
            end
        end
		

        function stateDot = flatEarth_EOM(obj, state, fB, mB)
            mass = state(1);
            vel = state(5:7);
            quat = state(8:11);
            omega = state(12:14);

            % preventing mass to go below massDry
            mass = max(mass, obj.rocketDynamics.massDry);

            [~, Ib] = obj.rocketDynamics.getCgMoi(mass);

            cBI = Utils.dirCosine(quat);
            cIB = cBI.';
            
            if obj.useGravity
                g = [0;0;-9.81];
            else
                g = [0;0;0];
            end

            fG = mass * g;

            fB = fB + cBI * fG;

            massDot = obj.rocketDynamics.getMDot(obj.currentTime);
            posDot = cIB * vel;
            velDot = fB / mass - cross(omega, vel);
            quatDot = Utils.quatDerivative(quat, omega);
            omegaDot = Ib\(mB - cross(omega, Ib*omega));
            stateDot = [massDot; posDot; velDot; quatDot; omegaDot];
        end
		%}
    end
end
