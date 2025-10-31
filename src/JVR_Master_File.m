%% Jet Vanes MasterFile Using GNCSIM 2.0
clear; clc; close all;

%% Instantiate Classes

% Setup Environment
environment = setupEnvironment();

% Setup CFD Aerodynamics
aerodynamics = setupAerodynamics();

% Setup Controller
% controller = setupController();
controller = -1;

% Setup BreakPoints
% breakPoints = setupBreakPoints(controller);
breakPoints = -1;

% Setup Navigation
navigation = setupNavigation();

% Rocket Dynamics
rocketDynamics = setupRocket(controller, navigation, aerodynamics);

% Setup Flight Simulation
flight = setupFlight(rocketDynamics, environment, breakPoints);

%% Run Flight
[tArr, xArr, FbArr, MbArr, stateEstimateArr] = flight.runSim();

% Plot Flight
flight.plotFlight(tArr, xArr);
flight.plotFlight(tArr, stateEstimateArr);

%% Analysis
% Still need to implement stuff there. 
n = 1; %For single flight
analysis = analysis_pkg.Analysis(tArr, xArr, n, FbArr, MbArr);
analysis.GenerateAllPlots();

%% Setup Functions

function RocketDynamics = setupRocket(controller, navigation, aerodynamics)

    % Create Rocket Params
    rocket_params.mass_wet = 40.912994; % kg
    rocket_params.mass_dry = 32.619994;
    rocket_params.CG_wet = 2.419762;
    rocket_params.CG_dry = 2.314814;

    wetRollMOI = 0.209731;  % Ixx
    wetPitchMOI = 24.655278;  % Iyy
    wetYawMOI = 24.656148;  % Izz
    dryRollMOI = 0.201846;
    dryPitchMOI = 22.301315;
    dryYawMOI = 22.301570;

    rocket_params.moi_wet = [wetRollMOI 0 0; 0 wetPitchMOI 0; 0 0 wetYawMOI];
    rocket_params.moi_dry = [dryRollMOI 0 0; 0 dryPitchMOI 0; 0 0 dryYawMOI];
    
    % Create Body
    body_params.body_length = 3.4;          % [m]
    body_params.rocket_diameter = 0.203;    % [m]

    % Motor params
    motor_params.burn_time = 13.1; % s
    motor_params.tc = [0.0, 0.457288;
        0.152, 0.314574;
        0.162, 40.2069;
        0.164, 148.3954;
        0.168, 598.904;
        0.172, 890.058;
        0.176, 995.096;
        0.178, 1158.031;
        0.182, 1342.306;
        0.19, 1556.716;
        0.222, 1959.8;
        0.234, 2050.53;
        0.254, 2111.01;
        0.308, 2119.9;
        0.356, 2073.29;
        0.47, 2091.07;
        0.528, 1989.543;
        0.582, 1825.825;
        0.648, 1733.526;
        0.782, 1658.07;
        0.892, 1701.725;
        0.914, 1633.231;
        0.966, 1669.558;
        1.7839, 1594.076;
        1.8239, 1551.156;
        3.1999, 1533.43;
        5.4538, 1378.986;
        6.9577, 1170.933;
        10.9216, 424.692;
        13.5895, 106.0499;
        14.6754, 12.60914;
        16.1354, 0.0;];
    motor_params.totalImpulse = 14583; % N-sec

    % Create rocket dynamics object    
    % trash below, please replace later with REAL dimensions
    %     fins_params.span       = 0.15;       % [m] placeholder
    %     fins_params.root_chord = 0.20;       % [m]
    %     fins_params.tip_chord  = 0.10;       % [m]
    %     fins_params.sweep      = 20*pi/180;  % [rad]
    %     fins_params.n_fins     = 4;
    %     fins = rocket_pkg.RocketFins(fins_params);
    % % REMOVE STUFF ABOVE

    %body = rocket_pkg.RocketBody(body_params);
    body = -1;
    rocket_fins = -1;
    RocketDynamics = rocket_pkg.RocketDynamics(body, rocket_fins, rocket_params, motor_params, controller, navigation, aerodynamics);
end

% Need to set up Flight to use environment()
function EnvironmentObj = setupEnvironment()
    % Assuming Location of GRITS Launch Site; 31.2732° N, 83.3538° W
    latitude = 31.2732; % deg
    longitude = 83.3538; % deg
    elevation = 75; % m
    
    % Use default location? (if not, then python is needed to generate the new location data)
    use_default = true;
    
    location = [latitude longitude elevation];
    EnvironmentObj = flight_pkg.Environment();
    EnvironmentObj.set_max_altitude(10000) % m
    EnvironmentObj.set_vm_mean(0.01) % m/s
    EnvironmentObj.set_vm_3sigma(0.01) % m/s
end

function [aerodynamics] = setupAerodynamics()
    aerodynamics = rocket_pkg.Aerodynamics();
    syms u v w rho CG p r
    Vars = [u v w rho CG p r];
    
    FAx_b_sym = (1/0.225)*(((0.64181 .* ((v/u)*(180/pi)) .* (0.64181 ./ (3.59306 + sqrt(1 - ((3.281*(sqrt(u^2 + v^2 + w^2)))./1097.40).^2)))) .* 0.5 .* (rho*0.00194) .* (3.281*(sqrt(u^2 + v^2 + w^2))).^2 .* 0.9375 + 0.5*(rho*0.00194)*(3.281*(sqrt(u^2 + v^2 + w^2)))^2*0.9375 * (2.61799)/(1+sqrt(1+0.797628*(1-((3.281*(sqrt(u^2 + v^2 + w^2)))/1116.45)^2))) * asin(r*(10.3264-(3.281*CG))/(3.281*(sqrt(u^2 + v^2 + w^2)))))*sin(v/u) + ((0.00253 * ((v/u)*(180/pi)).^2 + 0.13932) .* 0.5 .* (rho*0.00194) .* (3.281*(sqrt(u^2 + v^2 + w^2))).^2 .* 0.9375)*-1*cos(v/u));
    FAy_b_sym = (1/0.225)*(((0.64181 .* ((v/u)*(180/pi)) .* (0.64181 ./ (3.59306 + sqrt(1 - ((3.281*(sqrt(u^2 + v^2 + w^2)))./1097.40).^2)))) .* 0.5 .* (rho*0.00194) .* (3.281*(sqrt(u^2 + v^2 + w^2))).^2 .* 0.9375 + 0.5*(rho*0.00194)*(3.281*(sqrt(u^2 + v^2 + w^2)))^2*0.9375 * (2.61799)/(1+sqrt(1+0.797628*(1-((3.281*(sqrt(u^2 + v^2 + w^2)))/1116.45)^2))) * asin(r*(10.3264-(3.281*CG))/(3.281*(sqrt(u^2 + v^2 + w^2)))))*-1*cos(v/u) + ((0.00253 * ((v/u)*(180/pi)).^2 + 0.13932) .* 0.5 .* (rho*0.00194) .* (3.281*(sqrt(u^2 + v^2 + w^2))).^2 .* 0.9375)*-1*sin(v/u));
    FAz_b_sym = 0;
    MAx_b_sym = 1.36*((((4.*((2.*pi)./sqrt(1-((3.281*sqrt(u^2 + v^2 + w^2))./1116.4).^2)).*p)./(0.4688.*(3.281*sqrt(u^2 + v^2 + w^2)))).*0.1897).*0.5.*(rho*0.00194).*(3.281*sqrt(u^2 + v^2 + w^2)).^2.*0.4688);
    MAy_b_sym = 0;
    MAz_b_sym = 1.36*((((v/u)*(180/pi)) .* ((4.12421 ./ (27.81553 + 16.83430 .* sqrt(1 - ((3.281*sqrt(u^2 + v^2 + w^2))./1097.40).^2))) + -0.00014 .* ((v/u)*(180/pi)) + 0.01273)) * 0.5 * (rho*0.00194) * (3.281*sqrt(u^2 + v^2 + w^2)).^2 * 0.9375 + 0.5*(rho*0.00194)*(3.281*sqrt(u^2 + v^2 + w^2))^2*0.9375 * (2.61799)/(1+sqrt(1+0.797628*(1-((3.281*sqrt(u^2 + v^2 + w^2))/1116.45)^2))) * asin(r*(10.3264-(3.281*CG))/(3.281*sqrt(u^2 + v^2 + w^2))) * (10.3264-(3.281*CG)));
    
    FAx_b_func = matlabFunction(FAx_b_sym, vars=Vars);
    FAy_b_func = matlabFunction(FAy_b_sym, vars=Vars);
    FAz_b_func = matlabFunction(FAz_b_sym, vars=Vars);
    MAx_b_func = matlabFunction(MAx_b_sym, vars=Vars);
    MAy_b_func = matlabFunction(MAy_b_sym, vars=Vars);
    MAz_b_func = matlabFunction(MAz_b_sym, vars=Vars);
    
    aerodynamics.set_FAx_func(FAx_b_func);
    aerodynamics.set_FAy_func(FAy_b_func);
    aerodynamics.set_FAz_func(FAz_b_func);
    aerodynamics.set_MAx_func(MAx_b_func);
    aerodynamics.set_MAy_func(MAy_b_func);
    aerodynamics.set_MAz_func(MAz_b_func);

    aero_forces_moments = [FAx_b_sym, FAy_b_sym, FAz_b_sym, MAx_b_sym, MAy_b_sym, MAz_b_sym];
end

function breakPoints = setupBreakPoints()
    % TODO
end 

function controller = setupController() % Where should I implement trimpoints?
    % State: [u, v, w, p, q, r, q0, q1, q2, q3]
    Q = diag([10, 10, 10, 1, 1, 1, 1, 1, 1, 1]);   % Not Sure
    R = diag([1, 1, 1, 1]);                        % Not Sure
    
    % Create Jet Vanes Actuator

    % should set the bias and r24 here, not through motor params
    JetVanesActuator = JVR_files.JetVanesActuatorModel();
    JetVanesActuator.biasAng = [0; 0; 0; 0]; % [rad] Not Sure What Real Values Are
    JetVanesActuator.r24 = 0.0144;              % [m] Distance Between Vanes 2 and 4
    JetVanesActuator.location = 3.4;            % m (length of rocket)
    JetVanesActuator.q_ba = [1, 0, 0, 0];       % unit quaternion

    % Create LQR Controller
    controller = controller_pkg.LQR(Q, R, JetVanesActuator);
end

function nav = setupNavigation()
    % Navigation

    % Acc
    acc_params.VRW = -1;
    acc_params.leverArm = -1;
    acc_params.SF = -1;
    acc_params.bias = -1;

    acc_params.noiseCovariance = -1;
    acc_params.outputRate = 100;
    acc_params.outputProbability = 1;
    acc_params.observationMatrix = zeros(3);

   
    accelerometer = navigation_pkg.Accelerometer(acc_params);

    % GPS
    gps_params.refLLA = [33.776, -84.399, 306];
    gps_params.verticalAccuracy = 1;
    gps_params.horizontalAccuracy = 1;
    gps_params.velocityAccuracy = 1;

    gps_params.noiseCovariance = [8.4 0 0;
                                  0 4.2 0;
                                  0 0 4.2];
    gps_params.outputRate = 10;
    gps_params.outputProbability = 1;
    gps_params.observationMatrix = @(x)[1 0 0 0 0 0;
                                        0 1 0 0 0 0;
                                        0 0 1 0 0 0];
    gps = navigation_pkg.GPS(gps_params);

    % Gyro
    gyro_params.ARW = -1;
    gyro_params.bias = -1;

    gyro_params.noiseCovariance = -1;
    gyro_params.outputRate = 100;
    gyro_params.outputProbability = 1;
    gyro_params.observationMatrix = zeros(3);

    gyroscope = navigation_pkg.Gyroscope(gyro_params);

    nav_params.dt = .01;
    dt = nav_params.dt;
    nav_params.P = 0.1 * eye(6);

    nav_params.dfdx = @(dt, q)[1, dt*(2*q(1)^2 + 2*q(2)^2 - 1), 0,   dt*(2*q(1)*q(4) + 2*q(2)*q(3)), 0,  -dt*(2*q(1)*q(3) - 2*q(2)*q(4))
        0,                        1, 0,                        0, 0,                        0
        0,  -dt*(2*q(1)*q(4) - 2*q(2)*q(3)), 1, dt*(2*q(1)^2 + 2*q(3)^2 - 1), 0,   dt*(2*q(1)*q(2) + 2*q(3)*q(4))
        0,                        0, 0,                        1, 0,                        0
        0,   dt*(2*q(1)*q(3) + 2*q(2)*q(4)), 0,  -dt*(2*q(1)*q(2) - 2*q(3)*q(4)), 1, dt*(2*q(1)^2 + 2*q(4)^2 - 1)
        0,                        0, 0,                        0, 0,                        1];

    sigma_a = 10;

    nav_params.Q = sigma_a^2 .*[(dt^4)/4, (dt^3)/2,          0,           0,              0,          0;
                     (dt^3)/2,     dt^2,          0,           0,              0,          0;
                           0,         0,   (dt^4)/4,    (dt^3)/2,              0,          0;
                           0,         0,   (dt^3)/2,        dt^2,              0,          0;
                           0,         0,          0,           0,       (dt^4)/4,   (dt^3)/2;
                           0,         0,          0,           0,       (dt^3)/2,      dt^2];    

    nav = navigation_pkg.Navigation(nav_params, gps, gyroscope, accelerometer);
end


function flight = setupFlight(rocketDynamics, environment, breakPoints)
    % Flight
    flightParams.totalTime = 10;  % length of simulation [s]
    flightParams.dt = 0.01;  % time step [s]

    mass_0 = 20;
    pos_0 = [0, 0, 0];
    vel_0 = [0, 0, 100];
    quat_0 = Utils.euler_to_quat([0, -90, 0]).';
    omega_0 = [0, 0, 0];
    state_0 = [mass_0, pos_0, vel_0, quat_0, omega_0].';
    flightParams.x0 = state_0;

    breakPoints = -1;
    environment = -1;
    flight = flight_pkg.Flight(rocketDynamics, environment, breakPoints, flightParams);
end

