%% Master File
clear; clc; close all;

%% Instantiate Objects
rocket_dyn = setup_rocket();
%rocket_dyn.draw();

run_DATCOM(rocket_dyn)

flight = setup_flight(rocket_dyn);

flight.rocketDyn.draw()

%% Run Flight
[t, x, Fb, Mb] = flight.runSim();


%% Analysis
n = 1; %For single flight
%analysis = analysis_pkg.Analysis(t, x, n, Fb, Mb);
%analysis_pkg.GenerateAllPlots();

%flight.draw_traj()


%flight.plotFlight(t, x);

%% Functions
function rocket_dyn = setup_rocket()
    % Vehicle Properties
    rocket_params.longitudinal_moi = 9.15;  % [kg * m^2]
    rocket_params.rotational_moi = 0.112;  % [kg * m^2]
    rocket_params.cg_x = 1.84;  % [m]
    
    % Create Body
    body_params.nose_cone_length = 0.61;  % [m]
    body_params.body_length = 2.03;  % This does not include nose code [m]
    body_params.rocket_diameter = 0.157;  % [m]
    total_length = body_params.body_length + body_params.nose_cone_length;
    
    % Create Fins
    fin_params.num_fins = 4;  % []
    fin_params.root_chord = 0.203;  % [m]
    fin_params.tip_chord = 0.17;  % [m]
    fin_params.fin_span = 0.144;  % [m]
	fin_params.sweep_deg = 15.0;  % [deg]

    flight_con_params.mach_numbers = [0.2, 0.4, 0.6];        % Array of Mach numbers to test
    flight_con_params.altitudes = [0, 5000, 10000];       % Array of altitudes in feet
    flight_con_params.alphas_deg   = [-10, -8, -6, -4, -2, 0, 2, 4, 6, 8, 10]; % Array of angles of attack in degrees

    % Set wet mass
    rocket_params.mass_wet = 1;
    rocket_params.mass_dry = 1;
    rocket_params.moi_wet = eye(3);
    rocket_params.moi_dry = eye(3);
    rocket_params.CG_wet = 1;
    rocket_params.CG_dry = 1;
    
    % get the distance along body
    x_fin_ref = total_length - fin_params.root_chord;  % [m]
    
    % set location of where to put fins
    fin_params.translation = [x_fin_ref, body_params.rocket_diameter / 2, 0];  % [m]   
    
    % Motor
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
    motor_params.mass = 0.4;  % [kg]
    motor_params.x_ref = -1;  % [m]

    % Create rocket dynamics object
    body = rocket_pkg.RocketBody(body_params, flight_con_params);
    fins = rocket_pkg.RocketFins(fin_params, flight_con_params);
    
    rocket_dyn = rocket_pkg.RocketDynamics(body, fins, rocket_params, motor_params);
end


function run_DATCOM(rocket_dyn)
    % Run DATCOM and compute aerodynamics
    % Note that you can also load and save lookup tables
    rocket_dyn.compute_buildup()
    %rocket_dyn.draw_buildup()
    %rocket_dyn.save_buildup()
    %rocket_dyn.save_buildup_figs()
    %rocket_dyn.load_buildup_figs()
end

function environment = setup_environment()
    %environment_params.x1 = -1;
    %environment_params.x2 = -1;
    %environment = Environment(environment_params);
end

function controller = setup_controller()
    %Controller
    %controller = FinTabsController(rocket_dyn);
end

function nav = setup_navigation()
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

    gps_params.noiseCovariance = -1;
    gps_params.outputRate = 10;
    gps_params.outputProbability = .8;
    gps_params.observationMatrix = eye(3);


    gps = navigation_pkg.GPS(gps_params);

    % Gyro
    gyro_params.ARW = -1;
    gyro_params.bias = -1;

    gyro_params.noiseCovariance = -1;
    gyro_params.outputRate = 100;
    gyro_params.outputProbability = 1;
    gyro_params.observationMatrix = zeros(3);

    gyroscope = navigation_pkg.Gyroscope(gyro_params);

    nav_params.dt = .1;
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

function flight = setup_flight(rocket_dyn)
    % Flight
    flightParams.totalTime = 2;  % length of simulation [s]
    flightParams.dt = 0.01;  % time step [s]

    mass_0 = 20;
    pos_0 = [0, 0, 0];
    vel_0 = [0, 0, 100];
    quat_0 = Utils.euler_to_quat([0, -90, 0]).';
    omega_0 = [0, 0, 0];
    state_0 = [mass_0, pos_0, vel_0, quat_0, omega_0].';
    flightParams.x0 = state_0;

    nav = -1;
    breakPoints = -1;
    environment = -1;
    controller = -1;
    flight = flight_pkg.Flight(rocket_dyn, controller, environment, nav, breakPoints, flightParams);
end

function monte_carlo = setup_MonteCarlo()
    monte_carlo_params.set_wetRollMOI_3sigma = 1;
    monte_carlo = analysis_pkg.MonteCarlo(flight, monte_carlo_params);
end

function analysis = single_Flight_Analysis()
    
    
end









