
clear; clc; close all;

%% Vehicle Properties
rocket_params.longitudinal_moi = 9.15;  % [kg * m^2]
rocket_params.rotational_moi = 0.112;  % [kg * m^2]
rocket_params.cg_x = 1.84;  % [m]

%% Create Body
body_params.nose_cone_length = 0.61;  % [m]
body_params.ogive_radius = 1;  % gives shape of nose cone
body_params.body_length = 2.03;  % This does not include nose code [m]
body_params.rocket_diameter = 0.157;  % [m]
total_length = body_params.body_length + body_params.nose_cone_length;

%% Create Fins
fin_params.num_fins = 4;  % []
fin_params.root_chord = 0.203;  % [m]
fin_params.tip_chord = 0.17;  % [m]
fin_params.fin_height = 0.144;  % [m]
fin_params.sweep_length = 0.033;  % [m]

% get the distance along body
x_fin_ref = total_length - rocket_params.cg_x - fin_params.root_chord;  % [m]

% set location of where to put fins
fin_params.translation = [x_fin_ref, body_params.rocket_diameter / 2, 0];  % [m]

%% Motor
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


%% Create rocket dynamics object
body = RocketBody(body_params);
fins = RocketFins(fin_params);

rocket_dyn = RocketDynamics(body, fins, rocket_params, motor_params);
rocket_dyn.draw()

% Run DATCOM and compute aerodynamics
% Note that you can also load and save lookup tables
rocket_dyn.compute_buildup()
rocket_dyn.draw_buildup()
%rocket_dyn.save_buildup()
%rocket_dyn.save_buildup_figs()
%rocket_dyn.load_buildup_figs()

%% Environment
environment_params.x1 = -1;
environment_params.x2 = -1;
environment = Environment(environment_params);

%% Controller
controller = FinTabsController(rocket_dyn);

%% Navigation
nav_params.acc_noise = 0.01;
nav = Navigation(nav_params);

%% Flight
flight_params.sim_time = 20;  % length of simulation [s]
flight_params.dt = 0.01;  % time step [s]
flight = Flight(rocket_dyn, controller, environment, nav, flight_params);

x0 = "inital_conditions";
flight.run(x0)
flight.draw_traj()

%% add monte_carlo object to 
monte_carlo_params.set_wetRollMOI_3sigma = 1;
monte_carlo = MonteCarlo(flight, monte_carlo_params);

monte_carlo.run()











