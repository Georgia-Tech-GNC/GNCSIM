%% The Lorax (HPR-TVC) Master Script
clear; clc; close all;

%% Instantiate Classes
% Setup Environment
environment = setupEnvironment();

% Setup Controller
% controller = setupController();
controller = -1;

% Setup BreakPoints
% breakPoints = setupBreakPoints(controller);
breakPoints = -1;

% Rocket Dynamics
rocketDynamics = setupRocket(controller); 
runDATCOM(rocketDynamics);

% Setup Flight Simulation (sets up navigation too)
[flight, log] = setupFlight(rocketDynamics, environment, breakPoints); 

%% Run Flight
log = flight.runSim(log);
log.trim();

%% Analysis
% Enforce custom time bounds
analysis = analysisPkg.Analysis(log);
analysis.generateAllPlots();

datcomAnalysis = analysisPkg.DatcomAnalysis(rocketDynamics);
datcomAnalysis.drawForcesAndMoments([0, 12])

%% Functions
function rocketDynamics = setupRocket(controller)
    % Vehicle Properties
    rocketParams.longitudinalMoiWet = 0.61;  % [kg * m^2]
    rocketParams.rotationalMoiWet = 0.005;  % [kg * m^2]
	rocketParams.cgXWet = 0.71;  % [m]
	rocketParams.massWet = 3.32; % [kg]

	rocketParams.longitudinalMoiDry = 0.56;  % [kg * m^2]
    rocketParams.rotationalMoiDry = 0.005;  % [kg * m^2]
    rocketParams.cgXDry = 0.67;  % [m]
	rocketParams.massDry = 3.12; % [kg]

    % Create Body
    bodyParams.noseLength = 0.324;  % [m]
    bodyParams.bodyLength = 0.965;  % This does not include nose cone [m]
    bodyParams.rocketDiameter = 0.102;  % [m]
    totalLength = bodyParams.bodyLength + bodyParams.noseLength;
	bodyParams.translation = [0, 0, 0];
    bodyParams.cgXWet = rocketParams.cgXWet;

	
    % Create Fins
    finParams.numFins = 4;  % []
    finParams.rootChord = 0.1;  % [m]
    finParams.tipChord = 0.06;  % [m]
    finParams.finSpan = 0.056;  % [m]
	finParams.sweepDeg = 35.6;  % [deg]

    flightConParams.machNumbers = [0.02, 0.05, 0.08];        % Array of Mach numbers to test
    flightConParams.altitudes = [0, 300, 600];       % Array of altitudes in feet
    flightConParams.alphasDeg   = [-10, -8, -6, -4, -2, 0, 2, 4, 6, 8, 10]; % Array of angles of attack in degrees

    % get the distance along body
    xFinRef = totalLength - finParams.rootChord;  % [m]

    % set location of where to put fins
    finParams.translation = [xFinRef, bodyParams.rocketDiameter / 2, 0];  % [m]

    % Motor
    motorParams.burnTime = 10.0;
    tmp = load("+theLoraxFiles\thrust_curve_hpr_tvc.mat");
    motorParams.thrustCurve = tmp.thrust_curve_hpr_tvc;
    motorParams.totalImpulse = 403; % N-sec
    motorParams.tIgnition = 1.0;

    % Create rocket dynamics object
    body = rocketPkg.RocketBody(bodyParams, flightConParams);
    fins = rocketPkg.RocketFins(finParams, flightConParams);

	aerodynamics = []; % No CFD model
    
    % Set up parachutes
    parachuteObjects = [];

    % Main Parachute
    canopyDiameter1 = 0.914;           % m
    canopyMass1 = 0.0357027521 * pi* (canopyDiameter1/2)^2;              % kg
    dragCoefficient1 = 2.2;            % dimensionless
    deployAltitude1 = 1000;        % Deploys at apogee
    parachuteObjects(1) = rocketPkg.ParachuteDynamics("Main Parachute", canopyDiameter1, canopyMass1, dragCoefficient1,deployAltitude1);
    
    % Create rocketDynamics object
    rocketDynamics = rocketPkg.RocketDynamics(body, fins, rocketParams, motorParams, controller, parachuteObjects, aerodynamics);
end

function runDATCOM(rocketDynamics)
    % Run DATCOM and compute aerodynamics
    rocketDynamics.computeBuildup()
end

function environmentObj = setupEnvironment()
    environmentObj = flightPkg.Environment();
    environmentObj.setMaxAltitude(10000); % m
    environmentObj.setVmMean(0.01); % m/s
    environmentObj.setVm3Sigma(0.01); % m/s
end

function breakPoints = setupBreakPoints()
    % TODO
end 

function controller = setupController() % Where should I implement trimpoints?
    % State: [u, v, w, p, q, r, q0, q1, q2, q3]
    Q = diag([10, 10, 10, 1, 1, 1, 1, 1, 1, 1]);   % Not Sure
    R = diag([1, 1, 1, 1]);                        % Not Sure

    % Create LQR Controller
    controller = controllerPkg.LQR(Q, R, jetVanesActuator);
end

function nav = setupNavigation(dt)
    % Navigation

    % Acc
    accParams.VRW = -1;
    accParams.leverArm = -1;
    accParams.SF = -1;
    accParams.bias = -1;

    accParams.noiseCovariance = -1;
    accParams.outputRate = 100;
    accParams.outputProbability = 1;
    accParams.observationMatrix = zeros(3);


    accelerometer = navigationPkg.Accelerometer(accParams);

    % GPS
    gpsParams.refLLA = [31.2732, 83.3538, 75]; % Check
    gpsParams.verticalAccuracy = 1;
    gpsParams.horizontalAccuracy = 1;
    gpsParams.velocityAccuracy = .05;

    gpsParams.noiseCovariance = [8.4 0 0;
                                  0 4.2 0;
                                  0 0 4.2];
    gpsParams.outputRate = 10;
    gpsParams.outputProbability = 1;
    gpsParams.observationMatrix = @(x)[1 0 0 0 0 0;
                                        0 1 0 0 0 0;
                                        0 0 1 0 0 0];
    gps = navigationPkg.GPS(gpsParams);

    % Gyro
    gyroParams.ARW = -1;
    gyroParams.bias = -1;

    gyroParams.noiseCovariance = -1;
    gyroParams.outputRate = 100;
    gyroParams.outputProbability = 1;
    gyroParams.observationMatrix = zeros(3);

    gyroscope = navigationPkg.Gyroscope(gyroParams);

    navParams.dt = dt;
    navParams.P = 0.1 * eye(6);

    navParams.dfdx = @(dt) [1, 0, 0, dt, 0,  0;  
                            0, 1, 0, 0,  dt, 0;  
                            0, 0, 1, 0,  0,  dt; 
                            0, 0, 0, 1,  0,  0;  
                            0, 0, 0, 0,  1,  0;  
                            0, 0, 0, 0,  0,  1]; 
    sigmaA = 5;

    navParams.Q = sigmaA^2 .*[(dt^4)/2, 0, 0, (dt^3)/2, 0, 0;
                              0, (dt^4)/4, 0, 0, dt/2, 0;
                              0, 0, (dt^4)/4, 0, 0, dt/2;
                              dt/2, 0, 0, dt^2, 0, 0;
                              0, (dt^3)/2, 0, 0, dt^2, 0;
                              0, 0, (dt^3)/2, 0, 0, dt^2];

    nav = navigationPkg.Navigation(navParams, gps, gyroscope, accelerometer);
end

function [flight, log] = setupFlight(rocketDynamics, breakPoints, environment)
    % Flight
    flightParams.totalTime = 12;  % length of simulation [s]
    flightParams.dt = 0.01;  % time step [s]

    mass0 = rocketDynamics.massWet;
    pos0 = [0, 0, 0];
    vel0 = [0, 0, 0];
    quat0 = Utils.eulerToQuat([0, -90, 0]).';
    omega0 = [0, 0, 0];
    state0 = [mass0, pos0, vel0, quat0, omega0].';
    flightParams.x0 = state0;
    
    actuatorState0 = [0, 0, 0, 0];
    flightParams.del0 = actuatorState0;

    controlInput0 = [0, 0, 0];
    flightParams.u0 = controlInput0;

    navigation = setupNavigation(flightParams.dt);
    flight = flightPkg.Flight(rocketDynamics, environment, breakPoints, navigation, flightParams);
	
	% LOG
    maxSteps = round(flightParams.totalTime / flightParams.dt);
    stateNames = {'mass', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'q0', 'q1', 'q2', 'q3', 'wx', 'wy', 'wz'};
    controlInputNames = {'Mx', 'My', 'Mz'}; % control names are temporary
    actuatorStateNames = {'delta1', 'delta2', 'delta3', 'delta4'}; % fin deflections
	log = analysisPkg.Log(stateNames, controlInputNames, actuatorStateNames, maxSteps);
end

