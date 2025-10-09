%% Jet Vanes MasterFile Using GNCSim 2.0
clear; clc; close all;

% Rocket Dynamics
rocket_dyn = setup_rocket();

% Setup Environment
environment = setup_environment();

% Setup Controller
controller = setup_controller(rocket_dyn);

% Setup Navigation
nav = setup_navigation();

% Setup Flight Simulation
flight = setup_flight(rocket_dyn, controller, environment, nav);

% Run Simulation
[t_arr, x_arr] = flight.run_sim();

flight.plotFlight(t_arr, x_arr);

%% Setup Functions

function rocket_dyn = setup_rocket()
    % Create Rocket Params
    rocket_params = null;                   % Not Sure What This Is
    
    % Create Body
    body_params.nose_cone_length = 0;       % Not What Value To Put
    body_params.ogive_radius = 1;           % Not Sure About this
    body_params.body_length = 3.4;          % [m]
    body_params.rocket_diameter = 0.203;    % [m]
    
    % Create Fins
    fin_params.num_fins = 4;                % 4 Fins in JVR
    
    % Jet Vanes Parameters
    motor_params.biasAng = [0; 0; 0; 0];    % [rad] Not Sure What Real Values Are
    motor_params.r24 = 0.0144;              % [m] Distance Between Vanes 2 and 4
    motor_params.body_length = body_params.body_length; % [m]
    
    % Create rocket dynamics object
    body = RocketBody(body_params);
    fins = RocketFins(fin_params);
    
    rocket_dyn = RocketDynamics(body, fins, rocket_params, motor_params);
end

function environment = setup_environment()
    % No Environemnt Class RIght Now
end

function controller = setup_controller(rocket_dyn) % Where should I implement trimpoints?
    % State: [u, v, w, p, q, r, q0, q1, q2, q3]
    Q = diag([10, 10, 10, 1, 1, 1, 1, 1, 1, 1]);   % Not Sure
    R = diag([1, 1, 1, 1]);                        % Not Sure
    
    % Create Jet Vanes Actuator
    JetVanesActuator = JetVanesActuatorModel(rocket_dyn.motor_params);
    
    % Create LQR Controller
    controller = LQR(Q, R, JetVanesActuator);
end

function nav = setup_navigation()
    % Implement When Nav Class Finishes
end

function flight = setup_flight(rocket_dyn, controller, environment, nav)
    % Setup flight simulation parameters
    flight_params.dt = 0.01;                % [s] Time step
    flight_params.x0 = [0, 0, 0, 0.001, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0];  % Initial state from old JVR sim
    
    % Create flight object
    flight = Flight(rocket_dyn, controller, environment, nav, flight_params);
    flight.total_time = 20;                 % [s] Total Time for simulaion
end

