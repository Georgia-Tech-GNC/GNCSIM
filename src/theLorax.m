%% The Lorax (HPR-TVC) Master Script
clear; clc; close all;

%% Flags
runClosedLoop = true;

%% Instantiate Classes
% Setup Environment
environment = setupEnvironment();

% Rocket Dynamics
rocketDynamics = setupRocket();
runDATCOM(rocketDynamics);

% Controller   

if runClosedLoop
    useActuatorModel = false;
    controller = setupTVCPID();
else 
    controller = controllerPkg.Controller();
end

% Setup Flight Simulation (sets up navigation too)
[flight, log] = setupFlight(rocketDynamics, controller, environment);
flight.runClosedLoop = runClosedLoop;

%% Run Flight
tic;
log = flight.runSim(log);
elapsed_time = toc;
fprintf('Elapsed time: %.4f seconds\n', elapsed_time);
log.trim();

%% Analysis
analysis = analysisPkg.Analysis(log);
analysis.generateAllPlots();

datcomAnalysis = analysisPkg.DatcomAnalysis(rocketDynamics);

%% Functions
function rocketDynamics = setupRocket()
    % Vehicle Properties
    rocketParams.longitudinalMoiWet = 0.42;  % [kg * m^2]
    rocketParams.rotationalMoiWet = 0.005;  % [kg * m^2]
	rocketParams.massWet = 3.3; % [kg]
    momentArmWet = 0.311; % m thrust to cg

	rocketParams.longitudinalMoiDry = 0.35;  % [kg * m^2]
    rocketParams.rotationalMoiDry = 0.005;  % [kg * m^2]
	rocketParams.massDry = 3.05; % [kg]
    momentArmDry = 0.2924; % m thrust to cg


    % Create Body
    bodyParams.noseLength = 0.324;  % [m]
    bodyParams.bodyLength = 0.965;  % This does not include nose cone [m]
    bodyParams.rocketDiameter = 0.102;  % [m]
    totalLength = bodyParams.bodyLength + bodyParams.noseLength;

    rocketParams.cgXWet = totalLength - momentArmWet;  % [m] nose to Cg
    rocketParams.cgXDry = 0.67 - momentArmDry;  % [m] nose to Cg

	bodyParams.translation = [0, 0, 0];
    bodyParams.cgXWet = rocketParams.cgXWet;

    % Create Fins
    finParams.numFins = 4;  % []
    finParams.rootChord = 0.1;  % [m]
    finParams.tipChord = 0.06;  % [m]   
    finParams.finSpan = 0.056;  % [m]
	finParams.sweepDeg = 35.6;  % [deg]
    finParams.movable = false;
    finParams.name = 'STATIC_FIN';

    flightConParams.machNumbers = [0.02, 0.05, 0.08];        % Array of Mach numbers to test
    flightConParams.altitudes = [0, 300, 600];       % Array of altitudes in feet
    flightConParams.alphasDeg   = [-10, -8, -6, -4, -2, 0, 2, 4, 6, 8, 10]; % Array of angles of attack in degrees

    % get the distance along body
    xFinRef = totalLength - finParams.rootChord;  % [m]

    % set location of where to put fins
    finParams.translation = [xFinRef, bodyParams.rocketDiameter / 2, 0];  % [m]

    % Motor
    motor = setupMotor(rocketParams);

    % Create rocket dynamics object
    body = rocketPkg.RocketBody(bodyParams, flightConParams);
    fins = rocketPkg.RocketFins(finParams, flightConParams);

    % TVC Actuator Model
    motorPosition = totalLength;  % nozzle at aft end [m]
    allocationMatrix = [0, 1, 0; 0, 0, 1];  % 2x3: maps [roll, pitch, yaw] -> [delta_y, delta_z]
    timeConstant = 0.05;  % [s]
    limits.maxDeflection = 10;   % deg
    limits.maxRate = 100;       % deg/s
    actuatorModel = theLoraxFiles.TheLoraxActuatorModel(motor, motorPosition, timeConstant, allocationMatrix, limits);

    % Set up parachutes
    % Main Parachute
    canopyDiameter1 = 0.914;           % m
    canopyMass1 = 0.0357027521 * pi* (canopyDiameter1/2)^2;              % kg
    dragCoefficient1 = 2.2;            % dimensionless
    deployAltitude1 = 0;        % Deploys at apogee
    main = rocketPkg.ParachuteDynamics("Main Parachute", canopyDiameter1, canopyMass1, dragCoefficient1, deployAltitude1);

    % Create rocketDynamics object
    rocketDynamics = rocketPkg.RocketDynamics(body, fins, rocketParams, motor, actuatorModel, main);
end

function motor = setupMotor(rocketParams)
    tmp = load("+theLoraxFiles/thrust_curve_hpr_tvc.mat");
    thrustCurve  = tmp.thrust_curve_hpr_tvc;
    burnTime     = 10.0;
    totalImpulse = 403; % N-sec
    tIgnition    = 1.0;
    motorMass    = rocketParams.massWet - rocketParams.massDry;  % [kg] propellant mass

    motor = rocketPkg.Motor(thrustCurve, totalImpulse, burnTime, tIgnition, motorMass);
end

function runDATCOM(rocketDynamics)
    % Run DATCOM and compute aerodynamics
    rocketDynamics.computeBuildup()
end

function environmentObj = setupEnvironment()
    environmentObj = flightPkg.Environment();
    environmentObj.U0 = 0;
end


function controller = setupTVCPID()
    useActuatorModel = false;
    dt = 0.01;
    burnTime = 10;  % limit to burn duration
    controlTime = 23;  % full flight time
    
    % Time vector for log
    timeBurn = linspace(dt, burnTime, burnTime/dt);
    nBurn = length(timeBurn);
    
    % Time vector for reference attitudes
    timeFull = linspace(dt, controlTime, controlTime/dt);
    nFull = length(timeFull);

    % Reference attitude
    q_ref = Utils.eulerToQuat([0, -90, 0]);
    M = repmat(q_ref, 1, nFull);
    referenceAttitudes = double([timeFull; M]);

    % Fake Log for breakpoints
    stateNames         = {'x', 'y', 'z', 'vx', 'vy', 'vz', 'q0', 'q1', 'q2', 'q3', 'wx', 'wy', 'wz'};
    controlInputNames  = {'Tx', 'Ty', 'Tz'};
    actuatorStateNames = {'servo1', 'servo2'};
    nominalLog = analysisPkg.Log(stateNames, controlInputNames, actuatorStateNames, nBurn);

    dummyState    = zeros(length(stateNames), 1);
    dummyControl  = zeros(length(controlInputNames), 1);
    dummyActuator = zeros(length(actuatorStateNames), 1);

    for i = 1:nBurn
        nominalLog.initializeTimestep(timeBurn(i));
        nominalLog.add(timeBurn(i), 'states',         dummyState);
        nominalLog.add(timeBurn(i), 'stateDots',      dummyState);
        nominalLog.add(timeBurn(i), 'stateEstimates', dummyState);
        nominalLog.add(timeBurn(i), 'controlInputs',  dummyControl);
        nominalLog.add(timeBurn(i), 'actuatorStates', dummyActuator);
        nominalLog.add(timeBurn(i), 'aeroForces',     zeros(3,1));
        nominalLog.add(timeBurn(i), 'aeroMoments',    zeros(3,1));
        nominalLog.add(timeBurn(i), 'thrust',         0);
        nominalLog.add(timeBurn(i), 'actuatorForces', zeros(3,1));
        nominalLog.add(timeBurn(i), 'actuatorMoments',zeros(3,1));
    end
    nominalLog.trim();

    % PID Gains
    Kp =  .39250*[1; 1; 1];
    Ki = 0*[1; 1; 1];
    Kd = .18550*[1; 1; 1];

    bp = controllerPkg.BreakPoints(nominalLog, 1, 0.1);
    controller = controllerPkg.TVCPID(referenceAttitudes, Kp, Ki, Kd, useActuatorModel, bp);
end

function nav = setupNavigation(dt)
    % Navigation
    % https://www.analog.com/media/en/technical-documentation/data-sheets/adis16500.pdf
    % Page 3 Sensor Specifications for IMU
    % https://content.u-blox.com/sites/default/files/ZED-F9P-04B_DataSheet_UBX-21044850.pdf
    % Pagr 4-5 Sensor Specifications for GPS
    
    % Acc
    accParams.VRW = [0.033; 0.039; 0.039] * 1/60; % Which way will z-axis be mounted, assumed up, aligned with x in body
    accParams.leverArm = -1;
    accParams.SF = [0.01; 0.01; 0.01];
    accParams.bias = [0.05; 0.05; 0.05];
    
    accParams.noiseCovariance = -1;
    accParams.outputRate = 100;
    accParams.outputProbability = 1;
    accParams.observationMatrix = zeros(3);
    
    accelerometer = navigationPkg.Accelerometer(accParams);

     % Gyro
    gyroParams.ARW = [0.32; 0.29; 0.29] * pi/(180*60);
    gyroParams.bias = [4.85e-5; 4.85e-5; 4.85e-5];
    
    gyroParams.noiseCovariance = -1;
    gyroParams.outputRate = 100;
    gyroParams.outputProbability = 1;
    gyroParams.observationMatrix = zeros(3);
    
    gyroscope = navigationPkg.Gyroscope(gyroParams);
    
    % GPS
    gpsParams.refLLA = [31.2732, 83.3538, 75]; % Check
    gpsParams.verticalAccuracy = 8.4;
    gpsParams.horizontalAccuracy = 4.2;
    gpsParams.velocityAccuracy = .05;
    
    gpsParams.noiseCovariance = [gpsParams.verticalAccuracy 0 0;
        0 gpsParams.horizontalAccuracy 0;
        0 0 gpsParams.horizontalAccuracy];
    gpsParams.outputRate = 10;
    gpsParams.outputProbability = 0.75;
    gpsParams.observationMatrix = @(x)[1 0 0 0 0 0;
                                       0 1 0 0 0 0;
                                       0 0 1 0 0 0];
    gps = navigationPkg.GPS(gpsParams);
    
    % Magnetometer Params
    magParams.outputRate = 10;
    magParams.refLLA = [31.2732, 83.3538, 75]; % Check
    magParams.date = '00-01-2026';
    magnetometer = navigationPkg.Magnetometer(magParams);

    % Nav Params
    navParams.dt = dt;
    navParams.Pscale = 0.1;
    navParams.P = navParams.Pscale * eye(6);
    
    navParams.dfdx = @(dt) [1, 0, 0, dt, 0,  0;  
                            0, 1, 0, 0,  dt, 0;  
                            0, 0, 1, 0,  0,  dt; 
                            0, 0, 0, 1,  0,  0;  
                            0, 0, 0, 0,  1,  0;  
                            0, 0, 0, 0,  0,  1]; 
    sigmaA = 5;
    navParams.sigmaA = sigmaA;
    
    navParams.Q = sigmaA^2 .*[(dt^4)/2, 0, 0, (dt^3)/2, 0, 0;
                                  0, (dt^4)/4, 0, 0, dt/2, 0;
        0, 0, (dt^4)/4, 0, 0, dt/2;
        dt/2, 0, 0, dt^2, 0, 0;
        0, (dt^3)/2, 0, 0, dt^2, 0;
        0, 0, (dt^3)/2, 0, 0, dt^2];

    navParams.accBias = accParams.bias;
    navParams.accSF = accParams.SF;
    navParams.leverArm = accParams.leverArm;

    navParams.gyroBias = gyroParams.bias;
    
    nav = navigationPkg.Navigation(navParams, gps, gyroscope, accelerometer, magnetometer);

end

function [flight, log] = setupFlight(rocketDynamics, controller, environment)
    % Flight
    flightParams.totalTime = 200;  % length of simulation [s]
    flightParams.endState = flightPkg.FlightState.LANDED;
    flightParams.launchRailLength = 1.5;  % launch rail length [m]
    flightParams.dt = 0.01;  % time step [s]

    mass0 = rocketDynamics.massWet;
    pos0 = [0, 0, 0];
    vel0 = [0, 0, 0];
    quat0 = Utils.eulerToQuat([0, -90, 0]).';
    omega0 = [0, 0, 0];
    state0 = [mass0, pos0, vel0, quat0, omega0].';
    flightParams.x0 = state0;

    % TVC: 2-element actuator state [delta_y, delta_z]
    actuatorState0 = [0, 0];
    flightParams.del0 = actuatorState0;

    controlInput0 = [0, 0, 0];
    flightParams.u0 = controlInput0;

    navigation = setupNavigation(flightParams.dt);
    flight = flightPkg.Flight(rocketDynamics, environment, controller, navigation, flightParams);

	% LOG
    maxSteps = round(flightParams.totalTime / flightParams.dt);
    stateNames = {'mass', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'q0', 'q1', 'q2', 'q3', 'wx', 'wy', 'wz'};
    controlInputNames = {'Roll', 'Pitch', 'Yaw'};
    actuatorStateNames = {'delta_y', 'delta_z'};
	log = analysisPkg.Log(stateNames, controlInputNames, actuatorStateNames, maxSteps);
end
