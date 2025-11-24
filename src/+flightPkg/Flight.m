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

        useGravity = true
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
            log.initializeTimestep(obj.currentTime);

            rocketState = obj.rocketInitialState(:);
            rocketStateEstimate = obj.navigation.getStateEstimate(rocketState, zeros(14,1), obj.currentTime, log);
            rocketStateDot = obj.rocketModel(rocketState, log); % thrust starts at t=0s

            log.add(obj.currentTime, 'states', rocketState);
            log.add(obj.currentTime, 'statedots', rocketStateDot);
			log.add(obj.currentTime, 'stateestimates', rocketStateEstimate);

            for ii = 1:log.maxSteps - 1
                obj.currentTime = ii * obj.dt;
                log.initializeTimestep(obj.currentTime);

                rocketState = Utils.RK4(@obj.rocketModel, rocketState, obj.dt);
                quat = rocketState(8:11);
                quatNorm = Utils.normalizeQuat(quat);
                rocketState(8:11) = quatNorm;  

                rocketStateDot = obj.rocketModel(rocketState, log);
                rocketStateEstimate = obj.navigation.getStateEstimate(rocketState, rocketStateDot, obj.currentTime, log);

                log.add(obj.currentTime, 'states', rocketState);
                log.add(obj.currentTime, 'statedots', rocketStateDot);
                log.add(obj.currentTime, 'stateestimates', rocketStateEstimate);
            end
        end

        function xDot = rocketModel(obj, state, log)
            arguments
                obj
                state 
                log = []  % Optional, defaults to empty
            end

            % altitude = state(4);
            % disturbances.wind = obj.environment.getWindspeed(altitude);
            disturbances.wind = zeros(3, 1);

            [fB, mB] = obj.rocketDynamics.getForcesMoments(state, disturbances, obj.currentTime, log);
            xDot = obj.flatEarth_EOM(state, fB, mB);
        end

        % === TEMPORARY ===
        function xDot = rocketModelDeflection(obj, state, control, log)
			arguments
                obj
                state 
				control
                log = []  % Optional, defaults to empty
            end
            % Control: [Mx, My, Mz]
            altitude = state(4);
            disturbances.wind = zeros(3);
            obj.currentTime = 0;

            [fB, mB] = obj.rocketDynamics.getForcesMoments(state, disturbances, obj.currentTime, log);
            
            xDot = obj.flatEarth_EOM(state, fB, mB + control(:));

		    if ~isempty(log)
                log.add(obj.currentTime, 'statedots', xDot);
            end
        end
        
        function [stateDot] = flatEarth_EOM(obj, state, fB, mB)
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
    end
end
