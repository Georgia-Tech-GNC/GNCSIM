% Main Authors: David Reynolds (dreynolds45@gatech.edu)

classdef Flight < handle
    properties
        rocketDynamics
        environment
        breakPoints
        navigation
        flightParams

        totalTime
        currentTime
        currentXDot
        currentStateEstimate
        x0
        dt

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
            obj.x0 = obj.flightParams.x0;
            obj.totalTime = obj.flightParams.totalTime;
            
            %obj.rocket_dyn.calc_moi()
        end

        function log = runSim(obj, log)
            % This function propagates the dynamics using RK4

            % Need setters for this
            obj.currentTime = 0.0;
            x = obj.x0(:);

            log.initializeTimestep(obj.currentTime);
            log.add(obj.currentTime, 'states', x);

            obj.currentStateEstimate = obj.navigation.getStateEstimate(x, zeros(14,1), obj.currentTime, log);
            obj.currentXDot = obj.rocketModel(x, log); % thrust starts at t=0s

            log.add(obj.currentTime, 'statedots', obj.currentXDot);
			log.add(obj.currentTime, 'stateestimates', obj.currentStateEstimate);

            for ii = 1:log.maxSteps - 1
                k1 = obj.currentXDot;
                k2 = obj.rocketModel(x + (k1)*obj.dt/2);
                k3 = obj.rocketModel(x + (k2)*obj.dt/2);
                k4 = obj.rocketModel(x + (k3)*obj.dt);

                obj.currentTime = ii * obj.dt;

                x = x + obj.dt*((1/6)*k1 + (1/3)*k2 + (1/3)*k3 + (1/6)*k4);

                quat = x(8:11);
                quatNorm = Utils.normalizeQuat(quat);
                x(8:11) = quatNorm;     

                log.initializeTimestep(obj.currentTime);
                log.add(obj.currentTime, 'states', x);

                obj.currentXDot = obj.rocketModel(x, log);
                obj.currentStateEstimate = obj.navigation.getStateEstimate(x, obj.currentXDot, obj.currentTime, log);
                log.add(obj.currentTime, 'statedots', obj.currentXDot);
                log.add(obj.currentTime, 'stateestimates', obj.currentStateEstimate);
            end
        end

        function xDot = rocketModel(obj, state, log)
            arguments
                obj
                state 
                log = []  % Optional, defaults to empty
            end

            altitude = state(4);
            %disturbances.wind = obj.environment.getWindspeed(altitude);
            disturbances.wind = zeros(3);

            [fB, mB] = obj.rocketDynamics.getForcesMoments(state, disturbances, obj.currentTime, log);
            xDot = obj.flatEarth_EOM(state, fB, mB);

            % actuatorStateDot =
            % obj.rocketDynamics.controller.actuatorModel()
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

            mat_omega = [
                0, -omega(1), -omega(2), -omega(3);
                omega(1), 0, omega(3), -omega(2);
                omega(2), -omega(3), 0, omega(1);
                omega(3), omega(2), -omega(1), 0
            ];


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
            quatDot = 0.5 * mat_omega * quat;
            omegaDot = Ib\(mB - cross(omega, Ib*omega));
            stateDot = [massDot; posDot; velDot; quatDot; omegaDot];
        end
    end
end
