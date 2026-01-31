%% The Onceler (FIN-TABS) Master Script
clear; clc; close all;

%% Instantiate Classes
% Rocket Dynamics
rocketDynamics = setupRocket();
rocketDynamics.computeBuildup()

% Parachute Dynamics
parachuteDynamics = setupParachute();

% Setup Controller
useActuatorModel = false;
controller = setupDecoupledLQR(rocketDynamics, useActuatorModel);

% Setup Environment
environment = setupEnvironment();

% Setup Flight Simulation (sets up navigation too)
[flight, log] = setupFlight(rocketDynamics, controller, environment);
flight.runClosedLoop = true;

%% Run Flight
tic;
log = flight.runSim(log);
elapsed_time = toc;
fprintf('Elapsed time: %.4f seconds\n', elapsed_time);

log.trim();

% lti = controllerPkg.ReduceLTI(rocketDynamics);
% xStar = flight.rocketInitialState;
% xStar(5:7) = [20, 0, 0].';
% xStar(12:14) = [0, 0, 0].';
% lti.generateROM(xStar);
% lti.performComparison(log)

% log.saveToFile("theOncelerNominalFlightLog")

%% Analysis
analysis = analysisPkg.Analysis(log);
analysis.generateAllPlots();
%analysis.generateAllEstimatePlots();

%datcomAnalysis = analysisPkg.DatcomAnalysis(rocketDynamics);
%datcomAnalysis.drawForcesAndMoments([0, 12])


%% Functions

function rocketDyn = setupRocket()
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
	bodyParams.translation = [0, 0, 0];
	bodyParams.cgXWet = rocketParams.cgXWet;

    flightConParams.machNumbers = [0.2, 0.4, 0.6];        % Array of Mach numbers to test
    flightConParams.altitudes = [0, 5000, 10000];       % Array of altitudes in feet
    flightConParams.alphasDeg   = [-10, -8, -6, -4, -2, 0, 2, 4, 6, 8, 10]; % Array of angles of attack in degrees

    body = rocketPkg.RocketBody(bodyParams, flightConParams);
    
    % Create Fins
    finTabLength = 0.05;
    totalLength = bodyParams.bodyLength + bodyParams.noseLength;
	
    % Create Static Fins
    finParamsStatic.numFins = 4;  % []
    finParamsStatic.rootChord = 0.203 - finTabLength;  % [m]
    finParamsStatic.tipChord = 0.17 - finTabLength;  % [m]
    finParamsStatic.finSpan = 0.144;  % [m]
	finParamsStatic.sweepDeg = 12.9;  % [deg]
    finParamsStatic.movable = false;
    finParamsStatic.name = 'STATIC_FIN';
    
    % Create dynamics fins
    finParamsTabs.numFins = 4;
    finParamsTabs.rootChord = finTabLength;  % [m]
    finParamsTabs.tipChord = finTabLength;  % [m]
    finParamsTabs.finSpan = 0.144;  % [m]
	finParamsTabs.sweepDeg = 0.0;  % [deg]
    finParamsTabs.movable = true;
    finParamsTabs.name = 'DYNAMIC_FIN';
    
    % get the distance along body
    xFinRef = totalLength - finParamsStatic.rootChord;  % [m]

    % set location of where to put fins
    finParamsStatic.translation = [xFinRef-finTabLength, bodyParams.rocketDiameter / 2, 0];  % [m]
    finParamsTabs.translation = [totalLength-finTabLength, bodyParams.rocketDiameter / 2, 0];  % [m]
    
    rocketFinStatic = rocketPkg.RocketFins(finParamsStatic, flightConParams);
    rocketFinTabs = rocketPkg.RocketFins(finParamsTabs, flightConParams);

    % Motor
    motorParams.burnTime = 12.0;
    tmp = load("+theOncelerFiles\thrust_curve_fin_tabs.mat");
    motorParams.thrustCurve = tmp.thrust_curve_fin_tabs;
    motorParams.mass = 0.4;  % [kg]
    motorParams.xRef = -1;  % [m]
    motorParams.totalImpulse = 7561.0; % N-sec

    % CFD Aerodynamics not needed
	aerodynamics = [];

    % Actuator Model - uses rocketFinTabs

    % This is a mapping from roll, pitch and yaw to the four fin
    % deflections
    %u = [0; -1; 0; -1];  % positive roll (correct)
    %u = [-1; 0; 1; 0];  % positive pitch (correct)
  	%u = [0; 1; 0; -1];  % positive yaw (correct)

    allocationMatrix = [0, -1, 0, -1;
                        -1, 0, 1, 0;
                        0, 1, 0, -1].';
    timeConstant = 0.5; % [s]
    limits.maxDeflection = 15;        % deg
    limits.maxRate       = 300;       % deg/s
    actuatorModel = theOncelerFiles.TheOncelerActuatorModel(rocketFinTabs, timeConstant, allocationMatrix, limits);

    rocketDyn = rocketPkg.RocketDynamics(body, rocketFinStatic, rocketParams, motorParams, actuatorModel, aerodynamics);
end

function environment = setupEnvironment()
    environment = -1;
end

function controller = setupDecoupledLQR(rocketDynamics, useActuatorModel)
    % Compute Reference Attitudes
    dt = 0.01;  % or whatever your timestep is
    controlTime = 15;  % or whatever your total time is
    time = linspace(dt, controlTime, controlTime/dt);
    n = length(time);
    
    % Preallocate yaw angles
    yaw_deg = zeros(1, n);
    
    % Compute yaw for all time steps (vectorized)
    mask1 = time > 5;
    
    yaw_deg = 5 * ones(1, n);
    
    % Convert to radians
    yaw_rad = deg2rad(yaw_deg);
    pitch_rad = yaw_rad;
    
    % Build reference attitudes [4×n]: [time; roll; pitch; yaw]
    referenceAttitudes = [
        time;
        zeros(1, n);     % roll = 0
        zeros(1, n);       % pitch traj (same as yaw)
        zeros(1, n);       % yaw trajectory
    ];
    
    % Generate Breakpoints
    tmp = load("+theOncelerFiles/theOncelerNominalFlightLogQuat.mat");
    nominalTrajLog = tmp.log;
    numBreakPoints = 50;
    cumVelocityInterval = 0.1; % s
    breakPoints = controllerPkg.BreakPoints(nominalTrajLog, numBreakPoints, cumVelocityInterval);

    % Get controller
	Q2 = diag([1 1]);       
	R2 = 1;   	
	Q3 = diag([1 1 1]);
	R3 = 1; 
    reduceLTI = controllerPkg.ReduceLTI(rocketDynamics);

    controller = controllerPkg.DecoupledLQR(breakPoints, referenceAttitudes, Q2, Q3, R2, R3, reduceLTI, useActuatorModel);
    controller.populateBreakPoints();
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
    gpsParams.verticalAccuracy = 8.4;
    gpsParams.horizontalAccuracy = 4.2;
    gpsParams.velocityAccuracy = .05;
    
    gpsParams.noiseCovariance = [8.4 0 0;
        0 4.2 0;
        0 0 4.2];
    gpsParams.outputRate = 10;
    gpsParams.outputProbability = rand();
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

function [flight, log] = setupFlight(rocketDynamics, controller, environment)
    % Flight
    flightParams.totalTime = 21;  % length of simulation [s]
    flightParams.dt = 0.01;  % time step [s]
    
    mass0 = rocketDynamics.massWet;
    pos0 = [0, 0, 0];
    vel0 = [1, 0, 0];
    quat0 = Utils.eulerToQuat([0, 3, 0]).';
    omega0 = [0, 0, 0];
    state0 = [mass0, pos0, vel0, quat0, omega0].';
    flightParams.x0 = state0;

    actuatorState0 = [0, 0, 0, 0];
    flightParams.del0 = actuatorState0;
    
    navigation = setupNavigation(flightParams.dt);
    flight = flightPkg.Flight(rocketDynamics, environment, controller, navigation, flightParams);
    
    % LOG
    maxSteps = round(flightParams.totalTime / flightParams.dt);
    stateNames = {'mass', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'q0', 'q1', 'q2', 'q3', 'wx', 'wy', 'wz'};
    controlInputNames = {'Roll', 'Pitch', 'Yaw'}; % control names are temporary
    actuatorStateNames = {'fin1', 'fin2', 'fin3', 'fin4'}; % fin deflections
    log = analysisPkg.Log(stateNames, controlInputNames, actuatorStateNames, maxSteps);
end

function parachuteObjects =  setupParachute()
    parachuteObjects = -1;
    % drogueDiameter = .508; % m
    % drogueMass = 0.016811267; % kg
    % drogueCordLength = 1.52; %m
    % drogueDynamics = rocketPkg.ParachuteDynamics(drogueDiameter, drogueMass, drogueCordLength);
    % 
    % mainParachuteDiameter = 2.54; %m
    % mainParachuteMass = 0.3430292; %kg
    % mainParachuteCordLength = 1.78; %m
    % mainParachuteDynamics = rocketPkg.ParachuteDynamics(mainParachuteDiameter, mainParachuteMass, mainParachuteCordLength);
    % 
    % parachuteObjects = [mainParachuteDynamics, drogueDynamics];
end






