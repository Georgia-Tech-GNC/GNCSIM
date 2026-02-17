%% Syndrome (REACTION WHEEL) Master Script
clear; clc; close all;

paramsPath = fullfile(pwd, "+SyndromeFiles", "params.json");
params = jsondecode(fileread(paramsPath));

%% Instantiate Classes
% Rocket Dynamics (includes actuator model)
rocketDynamics = setupRocket(params);
rocketDynamics.computeBuildup();

% Setup Controller
useActuatorModel = true;
controller = setupController(params, useActuatorModel);

% Setup Environment
environment = setupEnvironment(params);

% Setup Flight Simulation (sets up navigation too)
[flight, log] = setupFlight(params, rocketDynamics, controller, environment);
flight.runClosedLoop = true;

%% Run Flight
tic;
log = flight.runSim(log);
elapsed_time = toc;
fprintf('Elapsed time: %.4f seconds\n', elapsed_time);

log.trim();

%% Analysis
analysis = analysisPkg.Analysis(log);
analysis.generateAllPlots();
analysis.generateAllEstimatePlots();

%% Functions

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

    rocketParams.longitudinalMoiWet = params.rocket.longitudinal_moi;
    rocketParams.rotationalMoiWet   = params.rocket.rotational_moi;
    rocketParams.cgXWet             = params.rocket.cg_x;
    rocketParams.massWet            = params.rocket.mass_wet;

    rocketParams.longitudinalMoiDry = params.rocket.longitudinal_moi;
    rocketParams.rotationalMoiDry   = params.rocket.rotational_moi;
    rocketParams.cgXDry             = params.rocket.cg_x;
    rocketParams.massDry            = params.rocket.mass_dry;

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

    tc = params.motor.tc;
    tc = reshape(tc, [], 2); % ensure Nx2
    motorParams.thrustCurve = tc;
    motorParams.mass        = params.motor.mass;
    motorParams.xRef        = params.motor.x_ref;

    % burnTime + totalImpulse are required by RocketDynamics
    motorParams.burnTime     = tc(end, 1);
    motorParams.totalImpulse = trapz(tc(:, 1), tc(:, 2));

    % ---- Actuator Model (Reaction Wheel) ----
    mustHave(params.reaction_wheel, "Iw");
    mustHave(params.reaction_wheel, "tau_max");
    mustHave(params.reaction_wheel, "w_max");
    mustHave(params.reaction_wheel, "w_soft");
    if ~isfield(params.reaction_wheel, "b"), params.reaction_wheel.b = 0.0; end

    actuatorModel = SyndromeFiles.SyndromeActuatorModel( ...
        params.reaction_wheel.Iw, ...
        params.reaction_wheel.tau_max, ...
        params.reaction_wheel.w_max, ...
        params.reaction_wheel.w_soft, ...
        params.reaction_wheel.b);

    % No CFD aerodynamics
    aerodynamics = [];

    rocketDyn = rocketPkg.RocketDynamics(body, rocketFins, rocketParams, motorParams, actuatorModel, aerodynamics);
end

function controller = setupController(params, useActuatorModel)
    mustHave(params, "controller");
    mustHave(params.controller, "kp");
    mustHave(params.controller, "ki");
    mustHave(params.controller, "kd");
    mustHave(params.controller, "dt");
    mustHave(params.controller, "tau_max");
    mustHave(params.controller, "gamma");

    controller = controllerPkg.RollRatePID( ...
        params.controller.kp, ...
        params.controller.ki, ...
        params.controller.kd, ...
        params.controller.dt, ...
        params.controller.tau_max, ...
        params.controller.gamma);

    controller.useActuatorModel = useActuatorModel;
end

% PLACEHOLDER
function environment = setupEnvironment(~)
    environment = -1;
end

function nav = setupNavigation(dt, navParams)
    % Pull from JSON
    accParams  = navParams.accel;
    gyroParams = navParams.gyro;

    accelerometer = navigationPkg.Accelerometer(accParams);
    gyroscope     = navigationPkg.Gyroscope(gyroParams);

    nav = navigationPkg.SimpleNavigation(dt, gyroscope, accelerometer);
end

function [flight, log] = setupFlight(params, rocketDynamics, controller, environment)
    mustHave(params, "controller");
    mustHave(params, "navigation");
    mustHave(params.navigation, "gyro");
    mustHave(params.navigation, "accel");

    dt = params.controller.dt;

    totalTime = 20;

    flightParams.totalTime = totalTime;
    flightParams.dt = dt;

    % Initial state (14x1): [mass, pos(3), vel(3), quat(4), omega(3)]
    mass0 = rocketDynamics.massWet;
    pos0  = [0, 0, 0];
    vel0  = [0, 0, 0];
    quat0 = Utils.eulerToQuat([0, -90, 0]).';
    omega0 = [0, 0, 0];
    flightParams.x0 = [mass0, pos0, vel0, quat0, omega0].';

    % Actuator initial state (1x1): wheel speed
    mustHave(params, "reaction_wheel");
    mustHave(params.reaction_wheel, "w0");
    flightParams.del0 = params.reaction_wheel.w0; % 1x1

    navigation = setupNavigation(flightParams.dt, params.navigation);

    flight = flightPkg.Flight(rocketDynamics, environment, controller, navigation, flightParams);

    % LOG (repo-standard)
    maxSteps = round(flightParams.totalTime / flightParams.dt);
    stateNames = {'mass', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'q0', 'q1', 'q2', 'q3', 'wx', 'wy', 'wz'};
    controlInputNames = {'Roll', 'Pitch', 'Yaw'};
    actuatorStateNames = {'omega_w'}; % 1-state actuator (wheel speed)
    log = analysisPkg.Log(stateNames, controlInputNames, actuatorStateNames, maxSteps);
end

function mustHave(s, fieldName)
    if ~isfield(s, fieldName)
        error("Syndrome:MissingParam", "Missing required field '%s'.", fieldName);
    end
end