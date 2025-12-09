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
            x = obj.x0(:);
            %u = [0; -1; 0; -1];  % positive roll (correct)
			%u = [0; 1; 0; -1];  % positive yaw (correct)
            %u = [-1; 0; 1; 0];  % positive pitch (correct)
            u = [0; 0; 0];
            
            % Initialize logger time step
            log.initializeTimestep(obj.currentTime);
            log.add(obj.currentTime, 'states', x);
            
            % Get state estimate
            obj.currentStateEstimate = obj.navigation.getStateEstimate(x, zeros(14,1), obj.currentTime, log);
            
            %controlDeflections = obj.rocketDynamics.

            obj.currentXDot = obj.rocketModel(x, u, log); % thrust starts at t=0s


            log.add(obj.currentTime, 'states', rocketState);
            log.add(obj.currentTime, 'statedots', rocketStateDot);
			      log.add(obj.currentTime, 'stateestimates', rocketStateEstimate);

            for ii = 1:log.maxSteps - 1
                k1 = obj.currentXDot;
                k2 = obj.rocketModel(x + (k1)*obj.dt/2, u);
                k3 = obj.rocketModel(x + (k2)*obj.dt/2, u);
                k4 = obj.rocketModel(x + (k3)*obj.dt, u);

                obj.currentTime = ii * obj.dt;
                log.initializeTimestep(obj.currentTime);

                rocketState = Utils.RK4(@obj.rocketModel, rocketState, obj.dt);
                quat = rocketState(8:11);
                quatNorm = Utils.normalizeQuat(quat);
                rocketState(8:11) = quatNorm;  

                rocketStateDot = obj.rocketModel(rocketState, log);
                rocketStateEstimate = obj.navigation.getStateEstimate(rocketState, rocketStateDot, obj.currentTime, log);

                obj.currentXDot = obj.rocketModel(x, u, log);
                obj.currentStateEstimate = obj.navigation.getStateEstimate(x, obj.currentXDot, obj.currentTime, log);
                log.add(obj.currentTime, 'statedots', obj.currentXDot);
                log.add(obj.currentTime, 'stateestimates', obj.currentStateEstimate);

            end
        end

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
        
        % need to explain what this is
        

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
