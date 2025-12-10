%% The Onceler (FIN-TABS) Master Script
clear; clc; close all;

%% Instantiate Classes
% Setup Environment
% environment = setupEnvironment();
environment = -1;
controller = -1;
% Rocket Dynamics

rocketDynamics = setupRocket(controller); 
rocketDynamics.computeBuildup()

% Setup BreakPoints
numBreakPoints = 20;
maxSpeed = 200;  % m/s
breakPoints = controllerPkg.BreakPoints(rocketDynamics, numBreakPoints, maxSpeed);
breakPoints.generateBreakPoints();
%breakPoints = -1;

% Setup Controller
%controller = controllerPkg.Controller(breakPoints);
controller = -1;

% Parachute Dynamics
%parachuteDynamics = setupParachute();

% Setup Flight Simulation (sets up navigation too)
[flight, log] = setupFlight(rocketDynamics, environment, breakPoints);

%% Run Flight

tic;
log = flight.runSim(log);
elapsed_time = toc;
fprintf('Elapsed time: %.4f seconds\n', elapsed_time);

log.trim();

lti = analysisPkg.ReduceLTI(flight);
xStar = flight.rocketInitialState;
xStar(5:7) = [20, 0, 0].';
xStar(12:14) = [0, 0, 0].';
lti.generateROM(xStar);
lti.performComparison(log)


%% Analysis
%analysis = analysisPkg.Analysis(log);
%analysis.generateAllPlots();
%analysis.generateAllEstimatePlots();

%datcomAnalysis = analysisPkg.DatcomAnalysis(rocketDynamics);
%datcomAnalysis.drawForcesAndMoments([0, 12])


%% Functions
function rocketDyn = setupRocket(controller)
    % Vehicle Properties
    rocketParams.longitudinalMoiWet = 13.33;  % [kg * m^2]
    rocketParams.rotationalMoiWet = 0.12;  % [kg * m^2]
    rocketParams.cgXWet = 1.98;  % [m]
    rocketParams.massWet = 29.04; % [kg]
    
    rocketParams.longitudinalMoiDry = 11.95;  % [kg * m^2]
    rocketParams.rotationalMoiDry = 0.11;  % [kg * m^2]
    rocketParams.cgXDry = 1.9;  % [m]
    rocketParams.massDry = 24.72; % [kg]
    
    % Create Body
    bodyParams.noseLength = 0.61;  % [m]
    bodyParams.bodyLength = 2.2856;  % This does not include nose code [m]
    bodyParams.rocketDiameter = 0.157;  % [m]
    totalLength = bodyParams.bodyLength + bodyParams.noseLength;
	bodyParams.translation = [0, 0, 0];
	bodyParams.cgXWet = rocketParams.cgXWet;

    finTabLength = 0.05;
	
    % Create Static Fins
    finParamsStatic.numFins = 4;  % []
    finParamsStatic.rootChord = 0.203 - finTabLength;  % [m]
    finParamsStatic.tipChord = 0.17 - finTabLength;  % [m]
    finParamsStatic.finSpan = 0.144;  % [m]
	finParamsStatic.sweepDeg = 12.9;  % [deg]
    finParamsStatic.movable = false;
    finParamsStatic.name = 'STATIC_FIN';

    finParamsTabs.numFins = 4;
    finParamsTabs.rootChord = finTabLength;  % [m]
    finParamsTabs.tipChord = finTabLength;  % [m]
    finParamsTabs.finSpan = 0.144;  % [m]
	finParamsTabs.sweepDeg = 0.0;  % [deg]
    finParamsTabs.movable = true;
    finParamsTabs.name = 'DYNAMIC_FIN';


    flightConParams.machNumbers = [0.2, 0.4, 0.6];        % Array of Mach numbers to test
    flightConParams.altitudes = [0, 5000, 10000];       % Array of altitudes in feet
    flightConParams.alphasDeg   = [-10, -8, -6, -4, -2, 0, 2, 4, 6, 8, 10]; % Array of angles of attack in degrees
    
    % get the distance along body
    xFinRef = totalLength - finParamsStatic.rootChord;  % [m]

    % set location of where to put fins
    finParamsStatic.translation = [xFinRef-finTabLength, bodyParams.rocketDiameter / 2, 0];  % [m]
    finParamsTabs.translation = [xFinRef+finParamsStatic.rootChord-finTabLength, bodyParams.rocketDiameter / 2, 0];  % [m]

    finParamsArr = [finParamsTabs, finParamsStatic];


    % Motor
    motorParams.burnTime = 12.0;
    tmp = load("+theOncelerFiles\thrust_curve_fin_tabs.mat");
    motorParams.thrustCurve = tmp.thrust_curve_fin_tabs;
    motorParams.mass = 0.4;  % [kg]
    motorParams.xRef = -1;  % [m]
    motorParams.totalImpulse = 7561.0; % N-sec
    
    % Create rocket dynamics object
    body = rocketPkg.RocketBody(bodyParams, flightConParams);
    fins = rocketPkg.RocketFins(finParamsArr, flightConParams);

	aerodynamics = [];

    rocketDyn = rocketPkg.RocketDynamics(body, fins, rocketParams, motorParams, controller, aerodynamics);
end


function environment = setupEnvironment()
    %environment_params.x1 = -1;
    %environment_params.x2 = -1;
    %environment = Environment(environment_params);
end

function controller = setupController()
    %Controller
    %controller = FinTabsController(rocket_dyn);
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
    
    navParams.dfdx = @(dt, q)[1, 0, 0, dt*(2*q(1)^2 + 2*q(2)^2 - 1), dt*(2*q(1)*q(4) + 2*q(2)*q(3)), -dt*(2*q(1)*q(3) - 2*q(2)*q(4));
        0, 1, 0, -dt*(2*q(1)*q(4) - 2*q(2)*q(3)), dt*(2*q(1)^2 + 2*q(3)^2 - 1), dt*(2*q(1)*q(2) + 2*q(3)*q(4));
        0, 0, 1, dt*(2*q(1)*q(3) + 2*q(2)*q(4)), -dt*(2*q(1)*q(2) - 2*q(3)*q(4)), dt*(2*q(1)^2 + 2*q(4)^2 - 1);
        0, 0, 0, 1, 0, 0;
        0, 0, 0, 0, 1, 0;
        0, 0, 0, 0, 0, 1];
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
    flightParams.totalTime = 15;  % length of simulation [s]
    flightParams.dt = 0.01;  % time step [s]
    
    mass0 = rocketDynamics.massWet;
    pos0 = [0, 0, 0];
    vel0 = [20, 0, 0];
    quat0 = Utils.eulerToQuat([0, -90, 0]).';
    omega0 = [0, .1, 0];
    state0 = [mass0, pos0, vel0, quat0, omega0].';
    flightParams.x0 = state0;
    
    navigation = setupNavigation(flightParams.dt);
    flight = flightPkg.Flight(rocketDynamics, environment, breakPoints, navigation, flightParams);
    
    % LOG
    maxSteps = round(flightParams.totalTime / flightParams.dt);
    stateNames = {'mass', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'q0', 'q1', 'q2', 'q3', 'wx', 'wy', 'wz'};
    controlInputNames = {'Mx', 'My', 'Mz'}; % control names are temporary
    log = analysisPkg.Log(stateNames, controlInputNames, maxSteps);
end



function parachuteObjects =  setupParachute()
    drogueDiameter = .508; % m
    drogueMass = 0.016811267; % kg
    drogueCordLength = 1.52; %m
    drogueDynamics = rocketPkg.ParachuteDynamics(drogueDiameter, drogueMass, drogueCordLength);
    
    mainParachuteDiameter = 2.54; %m
    mainParachuteMass = 0.3430292; %kg
    mainParachuteCordLength = 1.78; %m
    mainParachuteDynamics = rocketPkg.ParachuteDynamics(mainParachuteDiameter, mainParachuteMass, mainParachuteCordLength);
    
    parachuteObjects = [mainParachuteDynamics, drogueDynamics];
end






