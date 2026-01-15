% Main Authors: David Reynolds (dreynolds45@gatech.edu)
%               Karsten Caillet (kcaillet3@gatech.edu)

classdef Flight < handle
    properties
        rocketDynamics
        controller
        environment
        navigation
        flightParams

        totalTime
        currentTime
        cumVelocity
        dt

        rocketInitialState
        actuatorInitialState

        runClosedLoop = true
        useActuatorModel
	end
	
    methods
        function obj = Flight(rocketDynamics, environment, controller, navigation, flightParams)
            obj.rocketDynamics = rocketDynamics;
            obj.environment = environment;
            obj.controller = controller;
            obj.navigation = navigation;
            obj.flightParams = flightParams;

            obj.dt = obj.flightParams.dt;
            obj.rocketInitialState = obj.flightParams.x0;
            obj.actuatorInitialState = obj.flightParams.del0;
            obj.totalTime = obj.flightParams.totalTime;

            obj.useActuatorModel = controller.useActuatorModel;      
        end

        function log = runSim(obj, log)
            obj.currentTime = 0.0;
            obj.cumVelocity = 0.0;
            rocketState = obj.rocketInitialState(:);
            actuatorState = obj.actuatorInitialState(:);

            controlInput = [0, 0, 0].';
            disturbances.wind = [0, 0, 0].';
            
            % Initialize logger time step
            log.initializeTimestep(obj.currentTime);
            
            % Get state estimate
            rocketStateEstimate = obj.navigation.getStateEstimate(rocketState, zeros(14,1), obj.currentTime, log);

            if obj.useActuatorModel 
                rocketStateDot = obj.rocketDynamics.rocketModel(rocketState, actuatorState, disturbances, obj.currentTime, log); % thrust starts at t=0s
            else
                rocketStateDot = obj.rocketDynamics.rocketModelControlWrapper(rocketState, controlInput, disturbances, obj.currentTime, log);
            end

            log.add(obj.currentTime, 'states', rocketState);
            log.add(obj.currentTime, 'statedots', rocketStateDot);
			log.add(obj.currentTime, 'stateestimates', rocketStateEstimate);
            log.add(obj.currentTime, 'actuatorStates', actuatorState);
            log.add(obj.currentTime, 'controlInputs', controlInput);

            for ii = 1:log.maxSteps - 1

                obj.currentTime = ii * obj.dt;
                log.initializeTimestep(obj.currentTime);

                if obj.runClosedLoop
                    % Calculate cumulated velocity every interval
                    if mod(obj.currentTime, obj.controller.breakPoints.cumVelocityInterval) < obj.dt/2 
                        vx = rocketState(5);
                        vy = rocketState(6);
                        vz = rocketState(7);
                        vel = sqrt(vx.^2 + vy.^2 + vz.^2);
                        obj.cumVelocity = obj.cumVelocity + vel;
                    end
                end
                
                if obj.useActuatorModel
                    actuatorDyn = @(x) obj.rocketDynamics.actuatorModel.actuatorDynamics(controlInput, x);
                    actuatorState = Utils.RK4(actuatorDyn, actuatorState, obj.dt);
                    rocketDyn = @(x) obj.rocketDynamics.rocketModel(x, actuatorState, disturbances, obj.currentTime);
                else
                    rocketDyn = @(x) obj.rocketDynamics.rocketModelControlWrapper(x, controlInput, disturbances, obj.currentTime);
                end

                rocketState = Utils.RK4(rocketDyn, rocketState, obj.dt);
                quat = rocketState(8:11);
                quatNorm = Utils.normalizeQuat(quat);
                rocketState(8:11) = quatNorm;

                disturbances.wind = [0, 0, 0].';
                
                if obj.runClosedLoop
                    controlInput = obj.controller.getControl(rocketState, obj.cumVelocity, obj.currentTime);
                end

                if obj.useActuatorModel 
                    rocketStateDot = obj.rocketDynamics.rocketModel(rocketState, actuatorState, disturbances, obj.currentTime, log); 
                else
                    rocketStateDot = obj.rocketDynamics.rocketModelControlWrapper(rocketState, controlInput, disturbances, obj.currentTime, log);
                end

                rocketStateEstimate = obj.navigation.getStateEstimate(rocketState, rocketStateDot, obj.currentTime, log);

                log.add(obj.currentTime, 'states', rocketState);
                log.add(obj.currentTime, 'statedots', rocketStateDot);
                log.add(obj.currentTime, 'stateestimates', rocketStateEstimate);
                log.add(obj.currentTime, 'actuatorStates', actuatorState);
                log.add(obj.currentTime, 'controlInputs', controlInput);
            end
        end
    end
end
