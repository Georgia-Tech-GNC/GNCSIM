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

% Setup Navigation
navigation = setupNavigation();

% Rocket Dynamics
rocketDynamics = setupRocket(controller); 

% Setup Flight Simulation
[flight, log] = setupFlight(rocketDynamics, environment, breakPoints, navigation); 

%% Run Flight
log = flight.runSim(log);
log.trim();

%% Analysis
% Enforce custom time bounds
analysis = analysisPkg.Analysis(log, 0, 30);
analysis.displayPlots = true;

% Default filename 'Flight_Analysis_Report.pdf' in current directory
analysis.generatePdfFile();

%% Functions
function rocketDynamics = setupRocket(controller)
    % Vehicle Properties
    rocketParams.longitudinalMoiWet = 9.15;  % [kg * m^2]
    rocketParams.rotationalMoiWet = 0.112;  % [kg * m^2]
	rocketParams.cgXWet = .6;  % [m]
	rocketParams.massWet = 20;

	rocketParams.longitudinalMoiDry = 9.15 * 0.9;  % [kg * m^2]
    rocketParams.rotationalMoiDry = 0.112 * 0.9;  % [kg * m^2]
    rocketParams.cgXDry = .6;  % [m]
	rocketParams.massDry = 18;

    % Create Body
    bodyParams.noseLength = 0.61;  % [m]
    bodyParams.bodyLength = 2.03;  % This does not include nose code [m]
    bodyParams.rocketDiameter = 0.157;  % [m]
    totalLength = bodyParams.bodyLength + bodyParams.noseLength;
	bodyParams.translation = [0, 0, 0];
	
    % Create Fins
    finParams.numFins = 4;  % []
    finParams.rootChord = 0.203;  % [m]
    finParams.tipChord = 0.17;  % [m]
    finParams.finSpan = 0.144;  % [m]
	finParams.sweepDeg = 15.0;  % [deg]

    flightConParams.machNumbers = [0.2, 0.4, 0.6];        % Array of Mach numbers to test
    flightConParams.altitudes = [0, 5000, 10000];       % Array of altitudes in feet
    flightConParams.alphasDeg   = [-10, -8, -6, -4, -2, 0, 2, 4, 6, 8, 10]; % Array of angles of attack in degrees

    % get the distance along body
    xFinRef = totalLength - finParams.rootChord;  % [m]
	
	quarterChord = finParams.rootChord / 4;
	bodyParams.xWapprox = 0;
	bodyParams.cgXWet = rocketParams.cgXWet;

    % set location of where to put fins
    finParams.translation = [xFinRef, bodyParams.rocketDiameter / 2, 0];  % [m]
	%finParams.translation = [xFinRe, bodyParams.rocketDiameter / 2, 0];  % [m]

    % Motor
    motorParams.burnTime = 13.1;
    motorParams.thrustCurve = [0.0, 0.457288;
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
    motorParams.mass = 0.4;  % [kg]
    motorParams.xRef = -1;  % [m]
    motorParams.totalImpulse = 14583; % N-sec

    % Create rocket dynamics object
    body = rocketPkg.RocketBody(bodyParams, flightConParams);
    fins = rocketPkg.RocketFins(finParams, flightConParams);

	aerodynamics = []; % No CFD model

    rocketDynamics = rocketPkg.RocketDynamics(body, fins, rocketParams, motorParams, controller, aerodynamics);
end
