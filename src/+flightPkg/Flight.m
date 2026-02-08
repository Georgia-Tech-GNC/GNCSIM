% Main Authors: David Reynolds (dreynolds45@gatech.edu)
%               Karsten Caillet (kcaillet3@gatech.edu)

classdef Flight < handle
    properties (Constant)
        idx = StateIndices
    end
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

        flightState = flightPkg.FlightState.PRELAUNCH
        endState = flightPkg.FlightState.LANDED
        timeStateChange
        launchRailLength
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

            obj.endState = obj.flightParams.endState;
            obj.launchRailLength = obj.flightParams.launchRailLength;

            obj.useActuatorModel = controller.useActuatorModel;
        end

        function log = runSim(obj, log)
            obj.currentTime = 0.0;
            obj.cumVelocity = 0.0;
            rocketState = obj.rocketInitialState(:);
            actuatorState = obj.actuatorInitialState(:);

            controlInput = [0.0, 0.0, 0.0].';
            disturbances.wind = [0, 0, 0].';
            
            % Initialize logger time step
            log.initializeTimestep(obj.currentTime);
            
            % Get state estimate
            rocketStateEstimate = obj.navigation.getStateEstimate(rocketState, zeros(14,1), obj.currentTime, log);

            if obj.useActuatorModel
                rocketStateDot = obj.rocketDynamics.rocketModel(rocketState, actuatorState, disturbances, obj.currentTime, obj.flightState, log);
            else
                rocketStateDot = obj.rocketDynamics.rocketModelControlWrapper(rocketState, controlInput, disturbances, obj.currentTime, obj.flightState, log);
            end

            log.add(obj.currentTime, 'states', rocketState);
            log.add(obj.currentTime, 'statedots', rocketStateDot);
			log.add(obj.currentTime, 'stateestimates', rocketStateEstimate);
            log.add(obj.currentTime, 'actuatorStates', actuatorState);
            log.add(obj.currentTime, 'controlInputs', controlInput);

            ii = 0;
            while obj.flightState ~= obj.endState && ii < log.maxSteps - 1
                ii = ii + 1;
                obj.currentTime = ii * obj.dt;
                log.initializeTimestep(obj.currentTime);

                if obj.runClosedLoop
                    % Calculate cumulated velocity every interval
                    if mod(obj.currentTime, obj.controller.breakPoints.cumVelocityInterval) < obj.dt/2
                        vx = rocketState(obj.idx.vx);
                        vy = rocketState(obj.idx.vy);
                        vz = rocketState(obj.idx.vz);
                        vel = sqrt(vx.^2 + vy.^2 + vz.^2);
                        obj.cumVelocity = obj.cumVelocity + (vel * obj.controller.breakPoints.cumVelocityInterval);
                    end
                end

                if obj.useActuatorModel
                    actuatorDyn = @(x) obj.rocketDynamics.actuatorModel.actuatorDynamics(controlInput, x);
                    actuatorState = Utils.RK4(actuatorDyn, actuatorState, obj.dt);
                    rocketDyn = @(x) obj.rocketDynamics.rocketModel(x, actuatorState, disturbances, obj.currentTime, obj.flightState);
                else
                    rocketDyn = @(x) obj.rocketDynamics.rocketModelControlWrapper(x, controlInput, disturbances, obj.currentTime, obj.flightState);
                end

                rocketState = Utils.RK4(rocketDyn, rocketState, obj.dt);
                quat = rocketState(obj.idx.quat);
                quatNorm = Utils.normalizeQuat(quat);
                rocketState(obj.idx.quat) = quatNorm;

                disturbances.wind = [0, 0, 0].';

                if obj.runClosedLoop
                    controlInput = obj.controller.getControl(rocketState, obj.cumVelocity, obj.currentTime);
                end

                if obj.useActuatorModel
                    rocketStateDot = obj.rocketDynamics.rocketModel(rocketState, actuatorState, disturbances, obj.currentTime, obj.flightState, log);
                else
                    rocketStateDot = obj.rocketDynamics.rocketModelControlWrapper(rocketState, controlInput, disturbances, obj.currentTime, obj.flightState, log);
                end

                % Update flight state
                obj.updateFlightState(obj.currentTime, rocketState, rocketStateDot);

                rocketStateEstimate = obj.navigation.getStateEstimate(rocketState, rocketStateDot, obj.currentTime, log);

                log.add(obj.currentTime, 'states', rocketState);
                log.add(obj.currentTime, 'statedots', rocketStateDot);
                log.add(obj.currentTime, 'stateestimates', rocketStateEstimate);
                log.add(obj.currentTime, 'actuatorStates', actuatorState);
                log.add(obj.currentTime, 'controlInputs', controlInput);
            end
        end

        function updateFlightState(obj, time, rocketState, rocketStateDot)
            import flightPkg.FlightState

            switch obj.flightState
                case FlightState.PRELAUNCH
                    % Transition to LAUNCH_RAIL once ignition time is reached
                    if time >= obj.rocketDynamics.tIgnition
                        obj.transitionTo(FlightState.LAUNCH_RAIL, time);
                    end

                case FlightState.LAUNCH_RAIL
                    % Transition to POWERED_ASCENT once x position in body frame
                    % exceeds launch rail length
                    quat = rocketState(obj.idx.quat);
                    cBI = Utils.dirCosine(quat);

                    posENU = rocketState(obj.idx.pos);
                    posBody = cBI * posENU;

                    if posBody(1) > obj.launchRailLength
                        obj.transitionTo(FlightState.POWERED_ASCENT, time);
                    end

                case FlightState.POWERED_ASCENT
                    % Transition to COASTING_ASCENT once burnout time has passed
                    if time > (obj.rocketDynamics.burnTime + obj.rocketDynamics.tIgnition)
                        obj.transitionTo(FlightState.COASTING_ASCENT, time);
                    end

                case FlightState.COASTING_ASCENT
                    % Transition to DESCENT once z velocity in ENU is negative
                    % AND z acceleration in ENU is negative
                    velBody = rocketState(obj.idx.vel);
                    accBody = rocketStateDot(obj.idx.vel);

                    quat = rocketState(obj.idx.quat);
                    cBI = Utils.dirCosine(quat);
                    cIB = cBI.';

                    velENU = cIB * velBody;
                    accENU = cIB * accBody;

                    if velENU(3) < 0 && accENU(3) < 0
                        obj.transitionTo(FlightState.DESCENT, time);
                    end

                case FlightState.DESCENT
                    % Transition to LANDED once z position in ENU is below 1.0m
                    posENU = rocketState(obj.idx.pos);
                    if posENU(3) < 1.0
                        obj.transitionTo(FlightState.LANDED, time);
                    end

                otherwise
                    error('Unknown flight state: %s', string(obj.flightState));
            end
        end
        
        function transitionTo(obj, newState, t)
            fprintf('State transition: %s -> %s at t=%.2f\n', ...
                    string(obj.flightState), string(newState), t);
            obj.flightState = newState;
            obj.timeStateChange = t;
        end
    end
end
