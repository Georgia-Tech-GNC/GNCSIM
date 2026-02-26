function [log, params] = SyndromeRunOnce(kp, ki, kd, opts)
    paramsPath = fullfile(pwd, "+SyndromeFiles", "params.json");
    params = jsondecode(fileread(paramsPath));

    params.controller.kp = kp;
    params.controller.ki = ki;
    params.controller.kd = kd;

    % --- Scenario overrides (Monte Carlo) ---
    if exist('opts','var') && isstruct(opts) && isfield(opts,'scenario') && ~isempty(opts.scenario)
        params = applyScenarioOverrides(params, opts.scenario);
    end

    %% Instantiate Classes
    rocketDynamics = setupRocket(params);
    rocketDynamics.computeBuildup();

    useActuatorModel = true;
    controller = setupController(params, useActuatorModel);

    environment = setupEnvironment(params);

    [flight, log] = setupFlight(params, rocketDynamics, controller, environment);
    flight.runClosedLoop = true;

    log = flight.runSim(log);
    log.trim();
end

function rocketDyn = setupRocket(params)
    % ---- Required JSON sections ----
    mustHave(params, "rocket");
    mustHave(params, "body");
    mustHave(params, "fins");
    mustHave(params, "motor");
    mustHave(params, "flight");
    mustHave(params, "reaction_wheel");

    % ---- Rocket inertial / mass properties ----
    mustHave(params.rocket, "longitudinal_moi");
    mustHave(params.rocket, "rotational_moi");
    mustHave(params.rocket, "cg_x");
    mustHave(params.rocket, "mass_wet");
    mustHave(params.rocket, "mass_dry");
    mustHave(params.rocket, "rollDampLin");
    mustHave(params.rocket, "rollDampQuad");

    rocketParams.longitudinalMoiWet = params.rocket.longitudinal_moi;
    rocketParams.rotationalMoiWet   = params.rocket.rotational_moi;
    rocketParams.cgXWet             = params.rocket.cg_x;
    rocketParams.massWet            = params.rocket.mass_wet;

    rocketParams.longitudinalMoiDry = params.rocket.longitudinal_moi;
    rocketParams.rotationalMoiDry   = params.rocket.rotational_moi;
    rocketParams.cgXDry             = params.rocket.cg_x;
    rocketParams.massDry            = params.rocket.mass_dry;

    rocketParams.rollDampLin  = params.rocket.rollDampLin;
    rocketParams.rollDampQuad = params.rocket.rollDampQuad;

    % ---- Flight conditions (DATCOM grid) ----
    mustHave(params.flight, "mach_numbers");
    mustHave(params.flight, "altitudes_ft");
    mustHave(params.flight, "alphas_deg");

    flightConParams.machNumbers = params.flight.mach_numbers;
    flightConParams.altitudes   = params.flight.altitudes_ft;
    flightConParams.alphasDeg   = params.flight.alphas_deg;

    % ---- Body ----
    mustHave(params.body, "nose_cone_length");
    mustHave(params.body, "body_length");
    mustHave(params.body, "rocket_diameter");

    bodyParams.noseLength     = params.body.nose_cone_length; % [m]
    bodyParams.bodyLength     = params.body.body_length;      % [m]
    bodyParams.rocketDiameter = params.body.rocket_diameter;  % [m]
    bodyParams.translation    = [0, 0, 0];
    bodyParams.cgXWet         = rocketParams.cgXWet;

    body = rocketPkg.RocketBody(bodyParams, flightConParams);

    % ---- Fins ----
    mustHave(params.fins, "num_fins");
    mustHave(params.fins, "root_chord");
    mustHave(params.fins, "tip_chord");
    mustHave(params.fins, "fins_span");
    mustHave(params.fins, "sweep_deg");

    finParams.numFins   = params.fins.num_fins;
    finParams.rootChord = params.fins.root_chord;
    finParams.tipChord  = params.fins.tip_chord;
    finParams.finSpan   = params.fins.fins_span;
    finParams.sweepDeg  = params.fins.sweep_deg;
    finParams.movable   = false;
    finParams.name      = 'STATIC_FIN';

    totalLength = bodyParams.bodyLength + bodyParams.noseLength;
    xFinRef = totalLength - finParams.rootChord;
    finParams.translation = [xFinRef, bodyParams.rocketDiameter / 2, 0];

    rocketFins = rocketPkg.RocketFins(finParams, flightConParams);

    % ---- Motor ----
    mustHave(params.motor, "tc");
    mustHave(params.motor, "mass");
    mustHave(params.motor, "x_ref");

    tc = reshape(params.motor.tc, [], 2);
    motorParams.thrustCurve = tc;
    motorParams.mass        = params.motor.mass;
    motorParams.xRef        = params.motor.x_ref;

    motorParams.burnTime     = tc(end, 1);
    motorParams.totalImpulse = trapz(tc(:, 1), tc(:, 2));

    % ---- Actuator Model (Reaction Wheel) ----
    mustHave(params.reaction_wheel, "Iw");
    mustHave(params.reaction_wheel, "w_max");
    mustHave(params.reaction_wheel, "b");
    mustHave(params.reaction_wheel, "tau_c");

    mustHave(params.reaction_wheel, "Kt");
    mustHave(params.reaction_wheel, "Ke");
    mustHave(params.reaction_wheel, "R_wind");
    mustHave(params.reaction_wheel, "I_max");
    mustHave(params.reaction_wheel, "V_bus_nom");

    actuatorModel = SyndromeFiles.SyndromeActuatorModel( ...
        params.reaction_wheel.Iw, ...
        params.reaction_wheel.w_max, ...
        params.reaction_wheel.b, ...
        params.reaction_wheel.tau_c, ...
        params.reaction_wheel.Kt, ...
        params.reaction_wheel.Ke, ...
        params.reaction_wheel.R_wind, ...
        params.reaction_wheel.I_max, ...
        params.reaction_wheel.V_bus_nom);

    aerodynamics = []; % No CFD aerodynamics
    rocketDyn = rocketPkg.RocketDynamics(body, rocketFins, rocketParams, motorParams, actuatorModel, aerodynamics);
end

function controller = setupController(params, useActuatorModel)
    mustHave(params, "controller");
    mustHave(params.controller, "kp");
    mustHave(params.controller, "ki");
    mustHave(params.controller, "kd");
    mustHave(params.controller, "gamma");

    if ~isfield(params.controller, "beta")
        params.controller.beta = 0.8;
    end

    controller = controllerPkg.RollRatePID( ...
        params.controller.kp, ...
        params.controller.ki, ...
        params.controller.kd, ...
        params.controller.gamma, ...
        params.controller.beta);

    controller.useActuatorModel = useActuatorModel;
end

% PLACEHOLDER
function environment = setupEnvironment(~)
    environment = -1;
end

function nav = setupNavigation(dt, navParams)
    accelerometer = navigationPkg.Accelerometer(navParams.accel);
    gyroscope     = navigationPkg.Gyroscope(navParams.gyro);
    nav = navigationPkg.SimpleNavigation(dt, gyroscope, accelerometer);
end

function [flight, log] = setupFlight(params, rocketDynamics, controller, environment)
    mustHave(params, "navigation");
    mustHave(params, "flight");

    % Navigation required fields (unchanged)
    mustHave(params.navigation, "gyro");
    mustHave(params.navigation.gyro, "bias");
    mustHave(params.navigation.gyro, "biasRW_sigma");
    mustHave(params.navigation.gyro, "lpf_cutoff_hz");
    mustHave(params.navigation.gyro, "latency_s");
    mustHave(params.navigation.gyro, "noiseCovariance");
    mustHave(params.navigation.gyro, "outputRate");
    mustHave(params.navigation.gyro, "outputProbability");
    mustHave(params.navigation.gyro, "observationMatrix");

    mustHave(params.navigation, "accel");
    mustHave(params.navigation.accel, "VRW");
    mustHave(params.navigation.accel, "leverArm");
    mustHave(params.navigation.accel, "SF");
    mustHave(params.navigation.accel, "bias");
    mustHave(params.navigation.accel, "noiseCovariance");
    mustHave(params.navigation.accel, "outputRate");
    mustHave(params.navigation.accel, "outputProbability");
    mustHave(params.navigation.accel, "observationMatrix");

    % FreeRTOS-style dt jitter params (used by Flight.m)
    mustHave(params.flight, "dt_nom");
    mustHave(params.flight, "tick_s");
    mustHave(params.flight, "p_miss1");
    mustHave(params.flight, "p_miss2");
    mustHave(params.flight, "latency_max_s");
    % ESC telemetry noise (no latency) used by Flight.m for tau_max_meas
    if ~isfield(params.flight, "omega_w_sigma"), params.flight.omega_w_sigma = 0.0; end
    if ~isfield(params.flight, "omega_w_bias"),  params.flight.omega_w_bias  = 0.0; end
    if ~isfield(params.flight, "v_bus_sigma"),   params.flight.v_bus_sigma   = 0.0; end
    if ~isfield(params.flight, "v_bus_bias"),    params.flight.v_bus_bias    = 0.0; end
    if ~isfield(params.flight, "latency_mode"),   params.flight.latency_mode = "uniform"; end

    dt_nom = params.flight.dt_nom;

    totalTime = 20;

    flightParams.totalTime = totalTime;
    flightParams.dt = dt_nom;

    flightParams.tick_s = params.flight.tick_s;
    flightParams.p_miss1 = params.flight.p_miss1;
    flightParams.p_miss2 = params.flight.p_miss2;
    flightParams.latency_max_s = params.flight.latency_max_s;
    flightParams.latency_mode = params.flight.latency_mode;
    flightParams.omega_w_sigma = params.flight.omega_w_sigma;
    flightParams.omega_w_bias  = params.flight.omega_w_bias;
    flightParams.v_bus_sigma   = params.flight.v_bus_sigma;
    flightParams.v_bus_bias    = params.flight.v_bus_bias;

    % Roll disturbance torque injection (optional)
    if ~isfield(params.flight,"tau_dist_bias_x"),   params.flight.tau_dist_bias_x = 0.0; end
    if ~isfield(params.flight,"tau_dist_step_x"),   params.flight.tau_dist_step_x = 0.0; end
    if ~isfield(params.flight,"tau_dist_step_t0_s"),params.flight.tau_dist_step_t0_s = 0.0; end

    flightParams.tau_dist_bias_x    = params.flight.tau_dist_bias_x;
    flightParams.tau_dist_step_x    = params.flight.tau_dist_step_x;
    flightParams.tau_dist_step_t0_s = params.flight.tau_dist_step_t0_s;

    % Initial state (14x1): [mass, pos(3), vel(3), quat(4), omega(3)]
    mass0 = rocketDynamics.massWet;
    pos0  = [0, 0, 0];
    vel0  = [0, 0, 0];
    quat0 = Utils.eulerToQuat([0, -90, 0]).';
    omega0 = [0, 0, 0];

    % Optional MC initial roll rate
    if isfield(params.flight, "wx0")
        omega0(1) = params.flight.wx0;
    end

    flightParams.x0 = [mass0, pos0, vel0, quat0, omega0].';

    % Actuator initial state (wheel speed)
    mustHave(params, "reaction_wheel");
    mustHave(params.reaction_wheel, "w0");
    flightParams.del0 = params.reaction_wheel.w0;

    navigation = setupNavigation(flightParams.dt, params.navigation);

    flight = flightPkg.Flight(rocketDynamics, environment, controller, navigation, flightParams);

    % LOG
    dt_min = flightParams.dt; % dt_k >= dt_nom (lateness-only model)
    maxSteps = ceil(flightParams.totalTime / dt_min) + 1;

    stateNames = {'mass', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'q0', 'q1', 'q2', 'q3', 'wx', 'wy', 'wz'};
    controlInputNames = {'Roll', 'Pitch', 'Yaw'};
    actuatorStateNames = {'omega_w_true','omega_w_meas','V_bus_true','V_bus_meas','tau_max_meas', ...
                            'wx_meas','dt_k','tau_dist_x'};
    log = analysisPkg.Log(stateNames, controlInputNames, actuatorStateNames, maxSteps);
end

function mustHave(s, fieldName)
    if ~isfield(s, fieldName)
        error("Syndrome:MissingParam", "Missing required field '%s'.", fieldName);
    end
end

function params = applyScenarioOverrides(params, s)
    % --- Plant ---
    if isfield(s,'Ix_scale')
        params.rocket.rotational_moi = params.rocket.rotational_moi * s.Ix_scale;
    end
    if isfield(s,'rollDampLin_scale')
        params.rocket.rollDampLin = params.rocket.rollDampLin * s.rollDampLin_scale;
    end

    % --- Actuator ---
    if isfield(s,'Iw_scale')
        params.reaction_wheel.Iw = params.reaction_wheel.Iw * s.Iw_scale;
    end
    if isfield(s,'b_scale')
        params.reaction_wheel.b = params.reaction_wheel.b * s.b_scale;
    end
    if isfield(s,'tau_c_scale')
        params.reaction_wheel.tau_c = params.reaction_wheel.tau_c * s.tau_c_scale;
    end
    if isfield(s,'Imax_scale')
        params.reaction_wheel.I_max = params.reaction_wheel.I_max * s.Imax_scale;
    end
    if isfield(s,'Vbus_scale')
        params.reaction_wheel.V_bus_nom = params.reaction_wheel.V_bus_nom * s.Vbus_scale;
    end

    % --- Initial condition for roll rate ---
    if isfield(s,'wx0')
        params.flight.wx0 = s.wx0;
    end

    % --- Gyro (x-axis) ---
    if isfield(s,'gyro_bias_x')
        params.navigation.gyro.bias(1) = params.navigation.gyro.bias(1) + s.gyro_bias_x;
    end
    if isfield(s,'gyro_biasRW_scale')
        params.navigation.gyro.biasRW_sigma(1) = params.navigation.gyro.biasRW_sigma(1) * s.gyro_biasRW_scale;
    end
    if isfield(s,'gyro_noise_scale')
        scale2 = s.gyro_noise_scale.^2;
        params.navigation.gyro.noiseCovariance(1,1) = params.navigation.gyro.noiseCovariance(1,1) * scale2;
    end
    if isfield(s,'gyro_latency_s')
        params.navigation.gyro.latency_s = s.gyro_latency_s;
    end

    % --- Scheduler timing jitter (FreeRTOS-style) ---
    if isfield(s,'p_miss1')
        params.flight.p_miss1 = s.p_miss1;
    end
    if isfield(s,'p_miss2')
        params.flight.p_miss2 = s.p_miss2;
    end
    if isfield(s,'latency_max_s')
        params.flight.latency_max_s = s.latency_max_s;
    end
    if isfield(s,'latency_mode')
        params.flight.latency_mode = s.latency_mode;
    end
end