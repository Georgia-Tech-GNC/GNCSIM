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

        % Nominal scheduler period (seconds) for control / integration
        dt_nom

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

            % Nominal dt for 100 Hz control
            obj.dt_nom = obj.flightParams.dt;

            obj.rocketInitialState = obj.flightParams.x0;
            obj.actuatorInitialState = obj.flightParams.del0;
            obj.totalTime = obj.flightParams.totalTime;

            obj.useActuatorModel = controller.useActuatorModel;
        end

        function log = runSim(obj, log)
            % ----------------------------
            % Initialize sim state / time
            % ----------------------------
            obj.currentTime = 0.0;
            obj.cumVelocity = 0.0;

            rocketState = obj.rocketInitialState(:);
            actuatorState = obj.actuatorInitialState(:);

            controlInput = [0, 0, 0].';
            disturbances.wind = [0, 0, 0].';

            % Actuator model handle (for tau_max computation, V_bus truth, etc.)
            am = obj.rocketDynamics.actuatorModel;

            % Cum-velocity bookkeeping for variable dt
            cumVelInterval = obj.controller.breakPoints.cumVelocityInterval;
            cumVelTimer = 0.0;
            stopAtApogee = obj.getFieldDefault(obj.flightParams, 'stopAtApogee', false);
            minApogeeStopAltitude = obj.getFieldDefault(obj.flightParams, 'minApogeeStopAltitude', 0.0);
            minAscentVz = obj.getFieldDefault(obj.flightParams, 'minAscentVz', 0.0);
            hasSeenAscent = false;

            % ----------------------------
            % FreeRTOS-style dt jitter params (lateness-only)
            % ----------------------------
            tick_s   = obj.getFieldDefault(obj.flightParams, 'tick_s', 0.001);          % 1 ms tick
            p_miss1  = obj.getFieldDefault(obj.flightParams, 'p_miss1', 0.05);         % P(+1 tick)
            p_miss2  = obj.getFieldDefault(obj.flightParams, 'p_miss2', 0.005);        % P(+2 ticks)
            lat_max  = obj.getFieldDefault(obj.flightParams, 'latency_max_s', 0.0001); % max extra latency (s)
            lat_mode = obj.getFieldDefault(obj.flightParams, 'latency_mode', 'uniform'); % 'uniform' or 'none'


            % Normalize probabilities defensively
            p_miss1 = max(p_miss1, 0.0);
            p_miss2 = max(p_miss2, 0.0);
            p0 = max(0.0, 1.0 - (p_miss1 + p_miss2));
            psum = p0 + p_miss1 + p_miss2;
            if psum <= 0
                p0 = 1.0; p_miss1 = 0.0; p_miss2 = 0.0;
            else
                p0 = p0 / psum; p_miss1 = p_miss1 / psum; p_miss2 = p_miss2 / psum;
            end

            % ----------------------------
            % ESC telemetry "sensors" (noise-only, no latency)
            % ----------------------------
            sigma_w = obj.getFieldDefault(obj.flightParams, 'omega_w_sigma', 0.0); % [rad/s]
            bias_w  = obj.getFieldDefault(obj.flightParams, 'omega_w_bias',  0.0); % [rad/s]
            sigma_V = obj.getFieldDefault(obj.flightParams, 'v_bus_sigma',   0.0); % [V]
            bias_V  = obj.getFieldDefault(obj.flightParams, 'v_bus_bias',    0.0); % [V]

            % Injected disturbance torque (bias + step)
            tau_bias = obj.getFieldDefault(obj.flightParams, 'tau_dist_bias_x', 0.0);
            tau_step = obj.getFieldDefault(obj.flightParams, 'tau_dist_step_x', 0.0);
            t0_step  = obj.getFieldDefault(obj.flightParams, 'tau_dist_step_t0_s', 0.0);

            % ----------------------------
            % Initial log entry at t = 0
            % ----------------------------
            log.initializeTimestep(obj.currentTime);

            rocketStateEstimate = obj.navigation.getStateEstimate(rocketState, zeros(14,1), obj.currentTime, log);

            if obj.useActuatorModel
                rocketStateDot = obj.rocketDynamics.rocketModel(rocketState, actuatorState, disturbances, obj.currentTime, log);
            else
                rocketStateDot = obj.rocketDynamics.rocketModelControlWrapper(rocketState, controlInput, disturbances, obj.currentTime, log);
            end

            % Telemetry for logging at t=0 (not used for control at t=0 here)
            omega_w_true = actuatorState(1);
            V_bus_true   = am.V_bus_nom;

            omega_w_meas = omega_w_true + bias_w + sigma_w * randn();
            V_bus_meas   = max(V_bus_true + bias_V + sigma_V * randn(), 0.0);
            tau_max_meas = am.computeTauMax(omega_w_meas, V_bus_meas);

            % ----------------------------
            % Compute initial controlInput at t = 0 (so first interval is closed-loop)
            % ----------------------------
            if obj.runClosedLoop
                if isa(obj.controller, 'controllerPkg.RollRatePID') && obj.useActuatorModel
                    gyro_meas0 = obj.navigation.gyroscope.getMeasurement(rocketState, [], obj.currentTime); % 3x1
                    state_for_control0 = rocketState;
                    state_for_control0(obj.idx.omega) = gyro_meas0;

                    % Use nominal dt for the controller's internal discrete-time math at startup.
                    % (The first dt_k is not sampled until inside the loop.)
                    controlInput = obj.controller.getControl( ...
                        state_for_control0, obj.cumVelocity, obj.currentTime, obj.dt_nom, tau_max_meas);
                else
                    controlInput = obj.controller.getControl(rocketState, obj.cumVelocity, obj.currentTime);
                end
            end

            log.add(obj.currentTime, 'states', rocketState);
            log.add(obj.currentTime, 'statedots', rocketStateDot);
            log.add(obj.currentTime, 'stateestimates', rocketStateEstimate);

            disturbances.tau_roll_x = tau_bias + tau_step * double(obj.currentTime >= t0_step);

            gyro_meas0 = obj.navigation.gyroscope.getMeasurement(rocketState, [], obj.currentTime);
            wx_meas0 = gyro_meas0(1);
            dt_k0 = obj.dt_nom; % you have not sampled dt_k yet at t=0; use nominal
            tau_dist0 = disturbances.tau_roll_x; % if not set yet, set it once at t=0
            log.add(obj.currentTime, 'actuatorStates', [omega_w_true; omega_w_meas; V_bus_true; V_bus_meas; tau_max_meas; ...
                                                        wx_meas0; dt_k0; tau_dist0]);

            log.add(obj.currentTime, 'controlInputs', controlInput);

            % ----------------------------
            % Main simulation loop
            % ----------------------------
            step = 0;
            while (obj.currentTime < obj.totalTime) && (step < log.maxSteps - 1)
                step = step + 1;

                % ----------------------------
                % Sample control timestep dt_k (lateness-only)
                % dt_k = dt_nom + missed_ticks * tick + latency
                % ----------------------------
                missed_ticks = randsample([0, 1, 2], 1, true, [p0, p_miss1, p_miss2]);

                if strcmpi(lat_mode, 'none') || lat_max <= 0
                    latency_s = 0.0;
                else
                    latency_s = lat_max * rand(); % non-negative
                end

                dt_k = obj.dt_nom + missed_ticks * tick_s + latency_s;

                % Advance time
                nextTime = obj.currentTime + dt_k;
                if nextTime > obj.totalTime
                    dt_k = obj.totalTime - obj.currentTime;
                    nextTime = obj.totalTime;
                end

                % Roll disturbance torque about body X (bias + step)
                disturbances.tau_roll_x = tau_bias + tau_step * double(obj.currentTime >= t0_step);

                % ----------------------------
                % Cumulated velocity bookkeeping (variable dt)
                % ----------------------------
                if obj.runClosedLoop
                    cumVelTimer = cumVelTimer + dt_k;
                    while cumVelTimer >= cumVelInterval
                        vx = rocketState(obj.idx.vx);
                        vy = rocketState(obj.idx.vy);
                        vz = rocketState(obj.idx.vz);
                        vel = sqrt(vx.^2 + vy.^2 + vz.^2);
                        obj.cumVelocity = obj.cumVelocity + vel;
                        cumVelTimer = cumVelTimer - cumVelInterval;
                    end
                end

                % ----------------------------
                % Integrate actuator then rocket over dt_k
                % ----------------------------
                if obj.useActuatorModel
                    actuatorDyn = @(x) obj.rocketDynamics.actuatorModel.actuatorDynamics(controlInput, x);
                    actuatorState = Utils.RK4(actuatorDyn, actuatorState, dt_k);

                    rocketDyn = @(x) obj.rocketDynamics.rocketModel(x, actuatorState, disturbances, nextTime);
                else
                    rocketDyn = @(x) obj.rocketDynamics.rocketModelControlWrapper(x, controlInput, disturbances, nextTime);
                end

                rocketState = Utils.RK4(rocketDyn, rocketState, dt_k);
                rocketState(obj.idx.quat) = Utils.normalizeQuat(rocketState(obj.idx.quat));

                disturbances.wind = [0, 0, 0].';

                % Update time after propagation
                obj.currentTime = nextTime;
                log.initializeTimestep(obj.currentTime);

                % ----------------------------
                % Compute ESC telemetry once (used for BOTH control + logging)
                % ----------------------------
                omega_w_true = actuatorState(1);
                V_bus_true   = am.V_bus_nom;

                omega_w_meas = omega_w_true + bias_w + sigma_w * randn();
                V_bus_meas   = max(V_bus_true + bias_V + sigma_V * randn(), 0.0);

                tau_max_meas = am.computeTauMax(omega_w_meas, V_bus_meas);

                % ----------------------------
                % Closed-loop: compute control for next interval
                % ----------------------------
                if obj.runClosedLoop
                    if isa(obj.controller, 'controllerPkg.RollRatePID') && obj.useActuatorModel
                        % Gyro latency + noise!
                        gyro_meas = obj.navigation.gyroscope.getMeasurement(rocketState, [], obj.currentTime); % 3x1
                        wx_meas = gyro_meas(1);
                        tau_dist = disturbances.tau_roll_x;
                        state_for_control = rocketState;
                        state_for_control(obj.idx.omega) = gyro_meas;

                        controlInput = obj.controller.getControl(state_for_control, obj.cumVelocity, obj.currentTime, dt_k, tau_max_meas);
                    else
                        controlInput = obj.controller.getControl(rocketState, obj.cumVelocity, obj.currentTime);
                    end
                end

                % ----------------------------
                % Logging at currentTime
                % ----------------------------
                if obj.useActuatorModel
                    rocketStateDot = obj.rocketDynamics.rocketModel(rocketState, actuatorState, disturbances, obj.currentTime, log);
                else
                    rocketStateDot = obj.rocketDynamics.rocketModelControlWrapper(rocketState, controlInput, disturbances, obj.currentTime, log);
                end

                rocketStateEstimate = obj.navigation.getStateEstimate(rocketState, rocketStateDot, obj.currentTime, log);

                log.add(obj.currentTime, 'states', rocketState);
                log.add(obj.currentTime, 'statedots', rocketStateDot);
                log.add(obj.currentTime, 'stateestimates', rocketStateEstimate);
                log.add(obj.currentTime, 'actuatorStates', [omega_w_true; omega_w_meas; V_bus_true; V_bus_meas; tau_max_meas; ...
                                                            wx_meas; dt_k; tau_dist]);
                log.add(obj.currentTime, 'controlInputs', controlInput);

                if stopAtApogee
                    cBI = Utils.dirCosine(rocketState(obj.idx.quat));
                    cIB = cBI.';
                    vInertial = cIB * rocketState(obj.idx.vel);
                    vzInertial = vInertial(3);
                    altitude = rocketState(obj.idx.z);

                    if vzInertial > minAscentVz
                        hasSeenAscent = true;
                    end

                    if hasSeenAscent && altitude >= minApogeeStopAltitude && vzInertial <= 0
                        break;
                    end
                end
            end
        end
    end

    methods (Access = private)
        function v = getFieldDefault(~, s, fieldName, defaultVal)
            if isstruct(s) && isfield(s, fieldName)
                v = s.(fieldName);
            else
                v = defaultVal;
            end
        end
    end
end
