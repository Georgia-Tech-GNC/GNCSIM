%% The Onceler (FIN-TABS) Master Script
clear; clc; close all;

%% Instantiate Classes

% Rocket Dynamics
rocketDynamics = setupRocket();
rocketDynamics.computeBuildup()

% Nominal Trajectory
generateNominalTrajectory = false;
if generateNominalTrajectory
    nominalTrajectory = setupNominalTrajectory(rocketDynamics);
else
    nominalTrajectory.useThisTraj = false;
end

% Setup Controller
useActuatorModel = true;
controller = setupDecoupledLQR(rocketDynamics, useActuatorModel, nominalTrajectory);

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

%% MonteCarlo
uncertaintyParams = setupUncertaintyParams(rocketDynamics, flight);

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
    tmp = load("+theOncelerFiles/thrust_curve_fin_tabs.mat");
    motorParams.thrustCurve = tmp.thrust_curve_fin_tabs;
    motorParams.mass = 0.4;  % [kg]
    motorParams.xRef = -1;  % [m]
    motorParams.tIgnition = 1.0;
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
    timeConstant = 0.1; % [s]
    limits.maxDeflection = 15;        % deg
    limits.maxRate       = 300;       % deg/s
    actuatorModel = theOncelerFiles.TheOncelerActuatorModel(rocketFinTabs, timeConstant, allocationMatrix, limits);
    
    % Set Up Parachutes (linked list)
    firstParachute = setupParachute();

    rocketDyn = rocketPkg.RocketDynamics(body, rocketFinStatic, rocketParams, motorParams, actuatorModel, firstParachute, aerodynamics);
end

function environment = setupEnvironment()
    environment = -1;
end


function nominalTrajectory = setupNominalTrajectory(rocketDynamics)

    problem = struct();

    % Set up bounds on states
    mass0 = rocketDynamics.massWet;
    pos0 = [0, 0, 0];
    vel0 = [0.01, 0, 0];
    quat0 = Utils.eulerToQuat([0, -90, 0]).';
    omega0 = [0, 0, 0];
    state0 = [mass0, pos0, vel0, quat0, omega0].';

    problem.bounds.initialState.low = state0;
    problem.bounds.initialState.upp = state0;

    problem.bounds.state.low = [rocketDynamics.massDry, -inf*ones(1,2), 0, -inf*ones(1,2), 0, -1*ones(1,4), -inf*ones(1,3)]';
    problem.bounds.state.upp = [rocketDynamics.massWet, inf*ones(1,3), inf*ones(1,3), ones(1,4), inf*ones(1,3)]';

    massF = rocketDynamics.massDry;
    quatF = Utils.eulerToQuat([0, -90, 0]).';
    omegaF = [0,0,0];
    problem.bounds.finalState.low = [massF, -inf*ones(1,3), -inf*ones(1,3), quatF, omegaF]';
    problem.bounds.finalState.upp = [massF, inf*ones(1,3), inf*ones(1,3), quatF, omegaF]';

    % Set time bounds
    problem.bounds.initialTime.low = 0;
    problem.bounds.initialTime.upp = 0;
    problem.bounds.finalTime.low = 15; % TODO Is there a place where this variable is not hardcoded?
    problem.bounds.finalTime.upp = 15;

    % Set control bounds
    % TODO: Because of the linearization scheme for regulator control after
    % this, should the control bounds for the traj optimization be set to
    % less than the actual limits?
    problem.bounds.control.low = (-rocketDynamics.actuatorModel.limits.maxDeflection/2)*ones(3,1);
    problem.bounds.control.upp = (rocketDynamics.actuatorModel.limits.maxDeflection/2)*ones(3,1);

    % Make a guess at the initial trajectory
    problem.guess.time = [0, problem.bounds.finalTime.upp];  % (s)
    problem.guess.state = [ state0,  [massF; 0;0;2000; 200;0;0; quat0'; omega0'] ];
    problem.guess.control = [[0;0;0], [0;0;0]];
   
    % Desired Maneuver Details
    turnQuat = Utils.eulerToQuat([0, -100, 0]); % 10 degree turn
    turnTime = 8; % seconds
    maxAoA = 10; % max angle of attack is 10 degrees

    trajectoryOptimization = trajOptimizationPkg.TrajectoryOptimization(turnQuat, turnTime, maxAoA);

    % Note on dynamics function args: rocketModel(state, actuatorState, disturbances, time, flightState, log, useGravity, useThrust)
    problem.func.dynamics = @(t,x,u)( rocketDynamics.rocketModelTrajOptWrapper(x, u, [], t, flightPkg.FlightState.POWERED_ASCENT, [], true, true) ); 
    problem.func.pathObj = @(t,x,u) ( trajectoryOptimization.pathObjectiveFunction(t,x,u) ); 
    problem.func.bndObj = @(t0,x0,tF,xF) ( trajectoryOptimization.boundaryObjectiveFunction(t0,x0,tF,xF) ); 
    problem.func.pathCst = @(t,x,u) ( trajectoryOptimization.pathConstraintFunction(t,x,u) ); 
    problem.func.bndCst = @(t0,x0,tF,xF) ( trajectoryOptimization.boundaryConstraintFunction(t0,x0,tF,xF) ); 

    problem.options(1).method = 'hermiteSimpson';
    problem.options(1).defaultAccuracy = 'low';
    problem.options(1).nlpOpt.MaxFunEvals = 1e6;
    problem.options(1).nlpOpt.MaxIter = 250;
    problem.options(1).hermiteSimpson.nSegment = 8;

    %problem.options(2).method = 'hermiteSimpson';
    %problem.options(2).defaultAccuracy = 'high';
    %problem.options(2).nlpOpt.MaxFunEvals = 1e6;
    %problem.options(2).nlpOpt.MaxIter = 1e3;
    %problem.options(2).hermiteSimpson.nSegment = 60;
        % With nSegment = 60, it takes 128 seconds per iteration.

    % Call to OptimTraj
    addpath("+trajOptimizationPkg\OptimTraj\")
    soln = optimTraj(problem);

    trajectoryResolution = 250; % number of interpolated points in nominal traj

    nominalTrajectory.time = linspace(soln(end).grid.time(1),soln(end).grid.time(end),trajectoryResolution);
    nominalTrajectory.states = soln(end).interp.state(nominalTrajectory.data.time);
    nominalTrajectory.controlInputs = soln(end).interp.control(nominalTrajectory.data.time);
    nominalTrajectory.actuatorState = rocketDynamics.actuatorModel.allocationMatrix*nominalTrajectory.data.controlInputs;

    trajectoryOptimization.plotNominalTrajectory(nominalTrajectory)

    userChoice = input("Would you like to save this trajectory? [Y/N] ", "s");

    if (userChoice == "Y")
        userFilename = input("What would you like to call the file? Enter filename, as in filename.mat: ", "s");
        filename = ['+theOncelerFiles/' userFilename, '.mat'];

        userChoice = input("Would you like to use this nominal trajectory for the rest of the script? [Y/N] ", "s");
        if (userChoice == "Y")
            nominalTrajectory.useThisTraj = true;
        else
            nominalTrajectory.useThisTraj = false;
        end
        save(filename, 'nominalTrajectory');
    end
     

end

function controller = setupDecoupledLQR(rocketDynamics, useActuatorModel, nominalTrajectory)
    % Compute Reference Attitudes
    dt = 0.01;  % or whatever your timestep is
    controlTime = 15;  % or whatever your total time is
    time = linspace(dt, controlTime, controlTime/dt);
    n = length(time);
    
	% --- Define Trajectories in Degrees ---
	roll_deg  = zeros(1, n);
	pitch_deg = zeros(1, n);
	yaw_deg   = zeros(1, n);
	
	% Apply the step at 6 seconds
	% Logical mask for time >= 6 seconds
	mask_step1 = (time >= 4.0);
    mask_step2 = (time >= 10.0);
	
	% Set pitch to 12 degrees for all points where time >= 6
	yaw_deg(mask_step1) = 10.0;
    yaw_deg(mask_step2) = 2.0;

	roll_rad  = deg2rad(roll_deg);
	pitch_rad = deg2rad(pitch_deg);
	yaw_rad   = deg2rad(yaw_deg);
	
	% --- Build Reference Attitudes [4 x n] ---
	% Format: [Time; Roll; Pitch; Yaw]
	referenceAttitudes = [
    	time;
    	roll_rad;
    	pitch_rad;
    	yaw_rad
	];

    % Generate Breakpoints
    tmp = load("+theOncelerFiles/theOncelerNominalFlightLogQuat.mat");
    nominalTrajLog = tmp.log;
    numBreakPoints = 50;
    cumVelocityInterval = 0.1; % s
    breakPoints = controllerPkg.BreakPoints(nominalTrajLog, numBreakPoints, cumVelocityInterval);

    % Get controller
	Q2 = diag([1 1]);       
	R2 = 20;   	
	Q3 = diag([1 1 1]);
	R3 = 2; 
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
    % End conditions (whichever comes first)
    flightParams.totalTime = 150;  % max simulation length [s]
    flightParams.endState = flightPkg.FlightState.LANDED; % State to stop simulation

    % Flight state parameters
    flightParams.launchRailLength = 5.0;  % launch rail length [m]

    flightParams.dt = 0.01;  % time step [s]

    % Initial conditions
    mass0 = rocketDynamics.massWet;
    pos0 = [0, 0, 0];
    vel0 = [1, 0, 0];
	initAngle = Utils.eulerToQuat([0, 0, 0]).'; % yaw, pitch, roll
	quatRot = Utils.eulerToQuat([0, -80, 0]).';
    orientationLaunchRail = Utils.quatMult(initAngle, quatRot).';
    omega0 = [0, 0, 0];
    state0 = [mass0, pos0, vel0, orientationLaunchRail, omega0].';
    flightParams.x0 = state0;

    % Open Loop Default
    actuatorState0 = [0, 0, 0, 0];
    flightParams.del0 = actuatorState0;

    controlInput0 = [0, 0, 0];
    flightParams.u0 = controlInput0;
    
    navigation = setupNavigation(flightParams.dt);
    flight = flightPkg.Flight(rocketDynamics, environment, controller, navigation, flightParams);
    
    % LOG
    maxSteps = round(flightParams.totalTime / flightParams.dt);
    stateNames = {'mass', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'q0', 'q1', 'q2', 'q3', 'wx', 'wy', 'wz'};
    controlInputNames = {'Roll', 'Pitch', 'Yaw'}; % control names are temporary
    actuatorStateNames = {'fin1', 'fin2', 'fin3', 'fin4'}; % fin deflections
    log = analysisPkg.Log(stateNames, controlInputNames, actuatorStateNames, maxSteps);
end

function firstParachute = setupParachute()
    % Drogue chute (deploys at apogee)
    drogueDiameter = .508; % m
    drogueMass = 0.016811267; % kg
    drogueCD = 1.52;
    drogueDeployAlt = 0; % m (not used for first chute)
    drogue = rocketPkg.ParachuteDynamics("Drogue Parachute", drogueDiameter, drogueMass, drogueCD, drogueDeployAlt);

    % Main chute (deploys at altitude)
    mainDiameter = 2.54; % m
    mainMass = 0.3430292; % kg
    mainCD = .8;
    mainDeployAlt = 305; % m
    main = rocketPkg.ParachuteDynamics("Main Parachute", mainDiameter, mainMass, mainCD, mainDeployAlt);

    % Link parachutes: drogue -> main
    drogue.setNext(main);

    firstParachute = drogue;
end

function uncertaintyObj = setupUncertaintyParams(rocketDynamics, flight)

    numRuns = 100;
    uncertaintyObj = analysisPkg.Uncertainty(numRuns);

    uncertaintyObj.addVariable('caScaleFactor', rocketDynamics.body.scaleFactor, 1, 'normal');
    uncertaintyObj.addVariable('cmScaleFactor', rocketDynamics.body.momentScaleFactor, 1, 'normal');
    uncertaintyObj.addVariable('cnScaleFactor', rocketDynamics.body.scaleFactor, 1, 'normal');
    % Missing Parachute Cd
    uncertaintyObj.addVariable('wetMass', rocketDynamics.massWet, 1, 'normal');
    uncertaintyObj.addVariable('dryMass', rocketDynamics.massDry, 1, 'normal');
    uncertaintyObj.addVariable('cgXWet', rocketDynamics.cgWet, 1, 'normal');
    uncertaintyObj.addVariable('cgXDry', rocketDynamics.cgDry, 1, 'normal');
    uncertaintyObj.addVariable('longitudinalMoiWet', rocketDynamics.moiWet(2,2), 1, 'normal');
    uncertaintyObj.addVariable('rotationalMoiWet', rocketDynamics.moiWet(1,1), 1, 'normal');
    uncertaintyObj.addVariable('longitudinalMoiDry', rocketDynamics.moiDry(2,2), 1, 'normal');
    uncertaintyObj.addVariable('rotationalMoiDry', rocketDynamics.moiDry(1,1), 1, 'normal');
    uncertaintyObj.addVariable('totalImpulse', rocketDynamics.totalImpulse, 1, 'normal');
    uncertaintyObj.addVariable('burnTime', rocketDynamics.burnTime, 1, 'normal');
    uncertaintyObj.addVariable('tIgnition', rocketDynamics.tIgnition, 1, 'normal');
    uncertaintyObj.addVariable('noseLength', rocketDynamics.body.bodyParams.noseLength, 1, 'normal');
    uncertaintyObj.addVariable('bodyLength', rocketDynamics.body.bodyParams.bodyLength, 1, 'normal');
    uncertaintyObj.addVariable('rocketDiameter', rocketDynamics.body.bodyParams.rocketDiameter, 1, 'normal');
    uncertaintyObj.addVariable('maxDeflection', rocketDynamics.actuatorModel.limits.maxDeflection, 1, 'normal');
    uncertaintyObj.addVariable('maxRate', rocketDynamics.actuatorModel.limits.maxRate, 1, 'normal');
    uncertaintyObj.addVariable('timeConstant', rocketDynamics.actuatorModel.timeConstant, 1, 'normal');
    uncertaintyObj.addVariable('railLength', flight.flightParams.launchRailLength, 1, 'normal');
    uncertaintyObj.addVariable('railOrientation', 1, 1, 'normal');           

    % Generate samples 
    uncertaintyObj.generateLhsSamples();

end




