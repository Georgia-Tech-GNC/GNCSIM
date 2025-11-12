%% Master File
clear; clc; close all;

%% Instantiate Objects
rocketDyn = setupRocket();
%rocketDyn.draw();

runDATCOM(rocketDyn)

[flight, log] = setup_flight(rocketDyn);

%flight.rocketDynamics.draw()

%% Run Flight
flight.runSim(log);


%% Analysis
n = 1; %For single flight
%analysis = analysisPkg.Analysis(t, x, n, fb, mb);
%analysisPkg.GenerateAllPlots();

%flight.draw_traj()

%lti = analysisPkg.ReduceLTI(flight);
%xStar = flight.flightParams.x0;
%uStar = [0 0 0].';
%lti.calculateROM(xStar, uStar)

analyis = analysisPkg.Analysis(log);
analyis.generateAllPlots()

%% Functions
function rocketDyn = setupRocket()
    % Vehicle Properties
    rocketParams.longitudinalMoi = 9.15;  % [kg * m^2]
    rocketParams.rotationalMoi = 0.112;  % [kg * m^2]
    rocketParams.cgX = 1.84;  % [m]

    % Create Body
    bodyParams.noseLength = 0.61;  % [m]
    bodyParams.bodyLength = 2.03;  % This does not include nose code [m]
    bodyParams.rocketDiameter = 0.157;  % [m]
    totalLength = bodyParams.bodyLength + bodyParams.noseLength;

    % Create Fins
    finParams.numFins = 4;  % []
    finParams.rootChord = 0.203;  % [m]
    finParams.tipChord = 0.17;  % [m]
    finParams.finSpan = 0.144;  % [m]
	finParams.sweepDeg = 15.0;  % [deg]

    flightConParams.machNumbers = [0.2, 0.4, 0.6];        % Array of Mach numbers to test
    flightConParams.altitudes = [0, 5000, 10000];       % Array of altitudes in feet
    flightConParams.alphasDeg   = [-10, -8, -6, -4, -2, 0, 2, 4, 6, 8, 10]; % Array of angles of attack in degrees

    % Set wet mass
    rocketParams.massWet = 1;
    rocketParams.massDry = 1;
    rocketParams.moiWet = eye(3);
    rocketParams.moiDry = eye(3);
    rocketParams.cgWet = 1;
    rocketParams.cgDry = 1;

    % get the distance along body
    xFinRef = totalLength - finParams.rootChord;  % [m]

    % set location of where to put fins
    finParams.translation = [xFinRef, bodyParams.rocketDiameter / 2, 0];  % [m]

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

    controller = -1;
	aerodynamics = -1;

    rocketDyn = rocketPkg.RocketDynamics(body, fins, rocketParams, motorParams, controller, aerodynamics);

    rocketDyn.massWet = 20;
    rocketDyn.massDry = 18;
    rocketDyn.cgWet = 0.6;
    rocketDyn.cgDry = 0.4;
    rocketDyn.moiWet = 4 * eye(3);
    rocketDyn.moiDry = 3 * eye(3);

end


function runDATCOM(rocketDyn)
    % Run DATCOM and compute aerodynamics
    % Note that you can also load and save lookup tables
    rocketDyn.computeBuildup()
    %rocketDyn.drawBuildup()
    %rocketDyn.saveBuildup()
    %rocketDyn.saveBuildup_figs()
    %rocketDyn.loadBuildup_figs()
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
    gpsParams.refLLA = [33.776, -84.399, 306];
    gpsParams.verticalAccuracy = 1;
    gpsParams.horizontalAccuracy = 1;
    gpsParams.velocityAccuracy = 1;

    gpsParams.noiseCovariance = [8.4 0 0;
                                  0 4.2 0;
                                  0 0 4.2];
    gpsParams.outputRate = 10;
    gpsParams.outputProbability = .8;
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

    navParams.dt = .1;
    dt = navParams.dt;
    navParams.P = 0.1 * eye(6);

    navParams.dfdx = @(dt, q)[1, dt*(2*q(1)^2 + 2*q(2)^2 - 1), 0,   dt*(2*q(1)*q(4) + 2*q(2)*q(3)), 0,  -dt*(2*q(1)*q(3) - 2*q(2)*q(4))
        0,                        1, 0,                        0, 0,                        0
        0,  -dt*(2*q(1)*q(4) - 2*q(2)*q(3)), 1, dt*(2*q(1)^2 + 2*q(3)^2 - 1), 0,   dt*(2*q(1)*q(2) + 2*q(3)*q(4))
        0,                        0, 0,                        1, 0,                        0
        0,   dt*(2*q(1)*q(3) + 2*q(2)*q(4)), 0,  -dt*(2*q(1)*q(2) - 2*q(3)*q(4)), 1, dt*(2*q(1)^2 + 2*q(4)^2 - 1)
        0,                        0, 0,                        0, 0,                        1];

    sigmaA = 10;

    navParams.Q = sigmaA^2 .*[(dt^4)/4, (dt^3)/2,          0,           0,              0,          0;
                     (dt^3)/2,     dt^2,          0,           0,              0,          0;
                           0,         0,   (dt^4)/4,    (dt^3)/2,              0,          0;
                           0,         0,   (dt^3)/2,        dt^2,              0,          0;
                           0,         0,          0,           0,       (dt^4)/4,   (dt^3)/2;
                           0,         0,          0,           0,       (dt^3)/2,      dt^2];

    nav = navigationPkg.Navigation(navParams, gps, gyroscope, accelerometer);
end

function [flight, log] = setup_flight(rocketDyn)
    % Flight
    flightParams.totalTime = 20;  % length of simulation [s]
    flightParams.dt = 0.01;  % time step [s]

	maxSteps = round(flightParams.totalTime / flightParams.dt);
	
	% LOG
	log = analysisPkg.Log({'mass', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'q0', 'q1', 'q2', 'q3', 'wx', 'wy', 'wz'}, {'Mx', 'My', 'Mz'}, maxSteps);  % control names are temporary
    mass0 = 20;
    pos0 = [0, 0, 1];
    vel0 = [20, 0, 0];
    quat0 = Utils.eulerToQuat([0, 90, 0]).';
    omega0 = [0, 0, 0];
    state0 = [mass0, pos0, vel0, quat0, omega0].';
    flightParams.x0 = state0;

    breakPoints = -1;
    environment = -1;
	navigation = setup_navigation();
    flight = flightPkg.Flight(rocketDyn, environment, breakPoints, navigation, flightParams);
end

function monte_carlo = setup_MonteCarlo()
    monte_carlo_params.set_wetRollMOI_3sigma = 1;
    monte_carlo = analysisPkg.MonteCarlo(flight, monte_carlo_params);
end

function analysis = single_Flight_Analysis()
    
    
end









