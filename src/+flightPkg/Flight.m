% Main Authors: David Reynolds (dreynolds45@gatech.edu)
%               Karsten Caillet (kcaillet3@gatech.edu)

classdef Flight < handle & matlab.mixin.Copyable
    properties (Constant)
        idx StateIndices = StateIndices
    end
    properties
        rocketDynamics rocketPkg.RocketDynamics
        controller controllerPkg.Controller
        environment flightPkg.Environment
        navigation navigationPkg.Navigation
        flightParams struct

        totalTime (1,1) double {mustBeNonnegative}
        currentTime (1,1) double {mustBeNonnegative}
        cumVelocity (1,1) double {mustBeNonnegative}
        dt (1,1) double {mustBeNonnegative}

        rocketInitialState (:,1) double
        actuatorInitialState (:,1) double
        controlInputInitialState (:,1) double

        runClosedLoop (1,1) logical = true
        useActuatorModel (1,1) logical

        flightState (1,1) flightPkg.FlightState = flightPkg.FlightState.PRELAUNCH
        endState (1,1) flightPkg.FlightState = flightPkg.FlightState.LANDED
        timeStateChange (1,1) double
        launchRailLength (1,1) double
        groundAltitude (1,1) double
        launchLat (1,1) double
        launchLon (1,1) double
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
            obj.controlInputInitialState = obj.flightParams.u0;
            obj.totalTime = obj.flightParams.totalTime;

            obj.endState = obj.flightParams.endState;
            obj.launchRailLength = obj.flightParams.launchRailLength;

            obj.useActuatorModel = controller.useActuatorModel;
            
            % Setup Ground Altitude & Launch Coordinates
            if isfield(flightParams, 'launchLat') && isfield(flightParams, 'launchLon')
                obj.groundAltitude = 0; 
                try
                     % Offset starting altitude by the altitude of terrain relative to MSL
                     obj.groundAltitude = elevation(flightParams.launchLat, flightParams.launchLon);
                catch
                     obj.groundAltitude = 0;
                end
            else
                obj.groundAltitude = 0;
            end
            
            % Offset Initial Z Position by Ground Altitude (converting AGL to MSL roughly)
            obj.rocketInitialState(obj.idx.pos(3)) = obj.rocketInitialState(obj.idx.pos(3)) + obj.groundAltitude;
        end

        function log = runSim(obj, log)
            arguments
                obj
                log analysisPkg.Log
            end
            import flightPkg.FlightState
            
            obj.currentTime = 0.0;
            obj.cumVelocity = 0.0;
            rocketState = obj.rocketInitialState(:);
            actuatorState = obj.actuatorInitialState(:);
            controlInput = obj.controlInputInitialState(:);
            
            % Initialize Environment Conditions
            [T, a, P, rho] = obj.environment.getAtmosphere(obj.groundAltitude);
            wind = obj.environment.getWindVector(obj.groundAltitude);
            envConditions = flightPkg.AtmosphericConditions(rho, a, P, T, wind.');

            % Initialize logger time step
            log.initializeTimestep(obj.currentTime);
            
            % Get state estimate
            rocketStateEstimate = obj.navigation.getStateEstimate(rocketState, zeros(14,1), obj.currentTime, log);

            if obj.useActuatorModel
                rocketStateDot = obj.rocketDynamics.rocketModel(rocketState, actuatorState, envConditions, obj.currentTime, obj.flightState, true, true, log);
            else
                rocketStateDot = obj.rocketDynamics.rocketModelControlWrapper(rocketState, controlInput, envConditions, obj.currentTime, obj.flightState, true, true, log);
            end

            log.add(obj.currentTime, 'states', rocketState);
            log.add(obj.currentTime, 'statedots', rocketStateDot);
			log.add(obj.currentTime, 'stateestimates', rocketStateEstimate);
            log.add(obj.currentTime, 'actuatorStates', actuatorState);
            log.add(obj.currentTime, 'controlInputs', controlInput);
            log.add(obj.currentTime, 'flightState', obj.flightState);

            ii = 0;
            while obj.flightState ~= obj.endState && ii < log.maxSteps - 1
                ii = ii + 1;
                obj.currentTime = ii * obj.dt;
                log.initializeTimestep(obj.currentTime);

                % Update Environment Conditions
                alt = rocketState(obj.idx.z);
                [T, a, P, rho] = obj.environment.getAtmosphere(alt);
                wind = obj.environment.getWindVector(alt);
                envConditions = flightPkg.AtmosphericConditions(rho, a, P, T, wind.');

                switch obj.flightState
                    case FlightState.PRELAUNCH
                        % Just step time, keep state constant
                        rocketState = obj.rocketInitialState(:);
                        rocketStateDot = zeros(size(rocketState));
                        
                        % Log
                        log.add(obj.currentTime, 'states', rocketState);
                        log.add(obj.currentTime, 'statedots', rocketStateDot);
                        log.add(obj.currentTime, 'flightState', obj.flightState);

                        % Only update flight state (check for ignition)
                        obj.updateFlightState(obj.currentTime, rocketState, rocketStateDot);
                        
                    case FlightState.LAUNCH_RAIL
                        % Run open loop on launch rail
                        
                        % No wind on launch rail
                        envConditions.WindVector = [0;0;0];
                        
                        % Start accumulating velocity if closed loop
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
                        
                        rocketDyn = @(x) obj.rocketDynamics.rocketModel(x, obj.actuatorInitialState, envConditions, obj.currentTime, obj.flightState);
                        rocketState = Utils.RK4(rocketDyn, rocketState, obj.dt);
                        
                        % Clamp Orientation to Initial Launch Orientation
                        rocketState(obj.idx.quat) = obj.rocketInitialState(obj.idx.quat);
                        
                        rocketStateDot = obj.rocketDynamics.rocketModel(rocketState, obj.actuatorInitialState, envConditions, obj.currentTime, obj.flightState, true, true, log);                       
                        rocketStateEstimate = obj.navigation.getStateEstimate(rocketState, rocketStateDot, obj.currentTime, log);
                        
                        log.add(obj.currentTime, 'states', rocketState);
                        log.add(obj.currentTime, 'statedots', rocketStateDot);
                        log.add(obj.currentTime, 'stateestimates', rocketStateEstimate);
                        log.add(obj.currentTime, 'flightState', obj.flightState);

                        obj.updateFlightState(obj.currentTime, rocketState, rocketStateDot);

                    case {FlightState.POWERED_ASCENT, FlightState.COASTING_ASCENT}
                        % Full Simulation Logic
                        
                        if obj.runClosedLoop
                            % Calculate cumulated velocity every interval
                            if mod(obj.currentTime, obj.controller.breakPoints.cumVelocityInterval) < obj.dt/2
                                vx = rocketState(obj.idx.vx);
                                vy = rocketState(obj.idx.vy);
                                vz = rocketState(obj.idx.vz);
                                vel = sqrt(vx.^2 + vy.^2 + vz.^2);
                                obj.cumVelocity = obj.cumVelocity + (vel * obj.controller.breakPoints.cumVelocityInterval);
                            end

                            if obj.useActuatorModel
                                actuatorDyn = @(x) obj.rocketDynamics.actuatorModel.actuatorDynamics(controlInput, x);
                                actuatorState = Utils.RK4(actuatorDyn, actuatorState, obj.dt);
                                rocketDyn = @(x) obj.rocketDynamics.rocketModel(x, actuatorState, envConditions, obj.currentTime, obj.flightState);
                                log.add(obj.currentTime, 'actuatorStates', actuatorState);
                            else
                                rocketDyn = @(x) obj.rocketDynamics.rocketModelControlWrapper(x, controlInput, envConditions, obj.currentTime, obj.flightState);
                            end
                        else
                            rocketDyn = @(x) obj.rocketDynamics.rocketModel(x, obj.actuatorInitialState, envConditions, obj.currentTime, obj.flightState);
                        end

                        rocketState = Utils.RK4(rocketDyn, rocketState, obj.dt);
                        quat = rocketState(obj.idx.quat);
                        quatNorm = Utils.normalizeQuat(quat);
                        rocketState(obj.idx.quat) = quatNorm;

                        if obj.runClosedLoop
                            controlInput = obj.controller.getControl(rocketState, obj.cumVelocity, obj.currentTime);
                            log.add(obj.currentTime, 'controlInputs', controlInput);
                            if obj.useActuatorModel
                                rocketStateDot = obj.rocketDynamics.rocketModel(rocketState, actuatorState, envConditions, obj.currentTime, obj.flightState, true, true, log);
                            else
                                rocketStateDot = obj.rocketDynamics.rocketModelControlWrapper(rocketState, controlInput, envConditions, obj.currentTime, obj.flightState, true, true, log);
                            end
                        else
                            rocketStateDot = obj.rocketDynamics.rocketModel(rocketState, obj.actuatorInitialState, envConditions, obj.currentTime, obj.flightState, true, true, log);
                        end

                        rocketStateEstimate = obj.navigation.getStateEstimate(rocketState, rocketStateDot, obj.currentTime, log);

                        log.add(obj.currentTime, 'states', rocketState);
                        log.add(obj.currentTime, 'statedots', rocketStateDot);
                        log.add(obj.currentTime, 'stateestimates', rocketStateEstimate);
                        log.add(obj.currentTime, 'flightState', obj.flightState);

                        % Update flight state
                        obj.updateFlightState(obj.currentTime, rocketState, rocketStateDot);
                        
                    case FlightState.DESCENT
                        % Simplified Descent Logic
                         
                         rocketDyn = @(x) obj.rocketDynamics.rocketModel(x, obj.actuatorInitialState, envConditions, obj.currentTime, obj.flightState);
                         rocketState = Utils.RK4(rocketDyn, rocketState, obj.dt);

                         % Clamp Upright (-90 deg pitch wrt ENU)
                         rocketState(obj.idx.quat) = Utils.eulerToQuat([0, -90, 0]).';
                         rocketState(obj.idx.omega) = [0;0;0];
                         
                         rocketStateDot = obj.rocketDynamics.rocketModel(rocketState, obj.actuatorInitialState, envConditions, obj.currentTime, obj.flightState, true, true, log);

                         obj.updateFlightState(obj.currentTime, rocketState, rocketStateDot);

                         log.add(obj.currentTime, 'states', rocketState);
                         log.add(obj.currentTime, 'statedots', rocketStateDot);
                         log.add(obj.currentTime, 'flightState', obj.flightState);
                         
                    otherwise
                         error('Unknown flight state in runSim loop');
                end
            end
        end

        function updateFlightState(obj, time, rocketState, rocketStateDot)
            arguments
                obj
                time (1,1) double {mustBeNonnegative}
                rocketState (:,1) double
                rocketStateDot (:,1) double
            end
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
                    % Transition to LANDED once z position in ENU is below ground altitude + 1.0m
                    posENU = rocketState(obj.idx.pos);
                    if posENU(3) < obj.groundAltitude
                        obj.transitionTo(FlightState.LANDED, time);
                    end

                otherwise
                    error('Unknown flight state: %s', string(obj.flightState));
            end
        end
        
        function transitionTo(obj, newState, t)
            arguments
                obj
                newState flightPkg.FlightState
                t (1,1) double
            end
            fprintf('State transition: %s -> %s at t=%.2f\n', ...
                    string(obj.flightState), string(newState), t);
            obj.flightState = newState;
            obj.timeStateChange = t;
        end
    end

    methods (Access = protected)
        function cp = copyElement(obj)
            cp = copyElement@matlab.mixin.Copyable(obj);

            cp.rocketDynamics = flightPkg.Flight.copyHandleProperty(obj.rocketDynamics, "rocketDynamics");
            cp.controller = flightPkg.Flight.copyHandleProperty(obj.controller, "controller");
            cp.environment = flightPkg.Flight.copyHandleProperty(obj.environment, "environment");
            cp.navigation = flightPkg.Flight.copyHandleProperty(obj.navigation, "navigation");
        end
    end

    methods (Static, Access = private)
        function out = copyHandleProperty(in, propertyName)
            if isempty(in)
                out = in;
                return;
            end

            if isa(in, "matlab.mixin.Copyable")
                out = copy(in);
                return;
            end

            if ~isa(in, "handle")
                out = in;
                return;
            end

            try
                out = getArrayFromByteStream(getByteStreamFromArray(in));
            catch ME
                warning("Flight:ShallowCopyFallback", ...
                    "Could not deep copy property %s (%s). Reusing original handle. Error: %s", ...
                    propertyName, class(in), ME.message);
                out = in;
            end
        end
    end
end
