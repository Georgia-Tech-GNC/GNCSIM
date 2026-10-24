%% The Onceler (FIN-TABS) Master Script
clear; clc; close all;

%% Instantiate Classes

% Rocket Dynamics
rocketDynamics = setupRocket();
rocketDynamics.computeBuildup()

% Setup Environment
environment = setupEnvironment();

% Nominal Trajectory
generateNominalTrajectory = true;
if generateNominalTrajectory
    nominalTrajectory = setupNominalTrajectory(rocketDynamics, environment);
else
    nominalTrajectory.useThisTraj = false;
end
return
% Setup Controller
useActuatorModel = true;
controller = setupDecoupledLQR(rocketDynamics, useActuatorModel, nominalTrajectory, environment);

% Setup Flight Simulation (sets up navigation too)
[flight, log] = setupFlight(rocketDynamics, controller, environment);
flight.runClosedLoop = true;

%% Run Flight
runFlight = true;
if runFlight
    tic;
    log = flight.runSim(log);
    elapsed_time = toc;
    fprintf('Elapsed time: %.4f seconds\n', elapsed_time);
    log.trim();
    nominalLog = log;  % Save nominal log
end

%% MonteCarlo
runMonteCarlo = false;
numWorkers = 4;
numRuns = 10;

if runMonteCarlo
    % Close existing pool    
    delete(gcp('nocreate'));  
    parpool(numWorkers); 

    [mcFlight, mcLogTemplate] = setupFlight(rocketDynamics, controller, environment);
    mcFlight.runClosedLoop = true;
    
    uncertaintyParams = setupUncertaintyParams(rocketDynamics, environment, mcFlight);
    monteCarlo = setupMonteCarlo(uncertaintyParams, mcFlight, rocketDynamics, environment, mcLogTemplate, numRuns);

    tic;
    mcLogs = monteCarlo.runMonteCarlo();
    mc_elapsed_time = toc;
    fprintf('Monte Carlo elapsed time: %.4f seconds\n', mc_elapsed_time);
end

%% Analysis
if runFlight && ~isempty(nominalLog)
    if runMonteCarlo
        % If (Montecarlo + Nominal) generate just yaw angles plots for all runs
        mcLogs{end+1} = nominalLog;
        analysis = analysisPkg.Analysis(mcLogs);
        analysis.generateAllPlots();
        %analysis.generateYawAnglePlots();
    else
        % If only Nominal generate all plots
        analysis = analysisPkg.Analysis(nominalLog);
        analysis.generateAllPlots();
    end
else
        % If only Montecarlo is true generate all plots
        analysis = analysisPkg.Analysis(mcLogs);
        analysis.generateAllPlots();
end


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
    
    aeroSource = "datcom";   % "datcom" | "cfd"

    tabParams.sHat      = [0 0  0  0;
                           1 0 -1  0;
                           0 1  0 -1];   % outward span of tab i, body frame
    tabParams.deltaSign = +1;            % +delta right-handed about outward span

    aeroRef.Sref         = 0.019289796;  % [m^2] 
    aeroRef.Lref         = 0.156718;     % [m]   
    aeroRef.xRefFromAft = 0.938;        % [m]
    % Set Up Parachutes (linked list)
    firstParachute = setupParachute();

    rocketDyn = rocketPkg.RocketDynamics(body, rocketFinStatic, rocketParams, motorParams, actuatorModel, firstParachute);
end

function environment = setupEnvironment()
    environment = flightPkg.Environment();
end


function nominalTrajectory = setupNominalTrajectory(rocketDynamics, environment)

    problem = struct();

    % Zero-control ref
    % Generates a uncontrolled flight
    check = checkTrajOptDynamics(rocketDynamics, environment, tEnd=40);
    speed = vecnorm(check.states(5:7, :));
    i0 = find(check.time >= 3.0, 1); % controls are zero before 3 s
    iF = find(speed > 5, 1, 'last'); % end before apogee
    t0 = check.time(i0);
    tF = check.time(iF);
    state0 = check.states(:, i0);

    % Fixed start and end
    problem.bounds.initialTime.low = t0;   
    problem.bounds.initialTime.upp = t0;
    problem.bounds.finalTime.low = tF;     
    problem.bounds.finalTime.upp = tF;
    problem.bounds.initialState.low = state0;
    problem.bounds.initialState.upp = state0;

    % State bounds 
    % final state is free (same as the path bounds)
    % fix
    problem.bounds.state.low = [rocketDynamics.massDry - 0.1; -inf(2,1); 0; 0; -inf(2,1); -ones(4,1); -inf(3,1)];
    problem.bounds.state.upp = [rocketDynamics.massWet; inf(3,1); inf(3,1); ones(4,1); inf(3,1)];
    problem.bounds.finalState = problem.bounds.state;

    % Control bounds [deg]
    problem.bounds.control.low = -10 * ones(3,1);
    problem.bounds.control.upp =  10 * ones(3,1);

    % Guess: no control
    problem.guess.time = check.time(i0:iF);
    problem.guess.state = check.states(:, i0:iF);
    problem.guess.control = check.control(:, i0:iF);
    
    % Desired Maneuver Details 
    % Maybe make this not hardcoded? 
    % turnQuat = Utils.eulerToQuat([0, -100, 0]); % 10 degree turn
    % turnTime = 8; % seconds
    % maxAoA = 10; % max angle of attack is 10 degrees

    trajectoryOptimization = trajOptimizationPkg.TrajectoryOptimization();

    % FIX THIS
    groundAltitude = 0; % location of launch
    [T, a, P, rho] = environment.getAtmosphere(groundAltitude);
    % wind = environment.getWindVector(groundAltitude); 
    wind = [0, 0, 0]'; 
    envConditions = flightPkg.AtmosphericConditions(rho, a, P, T, wind.');
    % FIX THAT
    qT = zeros(4,1);
    
    % Note on dynamics function args: rocketModel(state, actuatorState, disturbances, time, flightState, log, useGravity, useThrust)
    problem.func.dynamics = @(t,x,u)( rocketDynamics.rocketModelTrajOptWrapper(x, u, envConditions, t, flightPkg.FlightState.POWERED_ASCENT, true, true) ); 
    problem.func.pathObj = @(t,x,u) ( trajectoryOptimization.pathObjectiveFunction(t,x,u) ); 
    problem.func.bndObj  = @(t0,x0,tF,xF) ( trajectoryOptimization.boundaryObjectiveFunction(t0,x0,tF,xF) );
    problem.func.pathCst = @(t,x,u) ( trajectoryOptimization.pathConstraintFunction(t,x,u) );
    problem.func.bndCst  = @(t0,x0,tF,xF) ( trajectoryOptimization.boundaryConstraintFunction(t0,x0,tF,xF) );


    problem.options(1).method = 'hermiteSimpson'; % control input is quadratic, state is cubic subject to dynamic constraint -> expected trajectory between nodes
    problem.options(1).defaultAccuracy = 'low'; % number of nodes (overridden by nsegment?)
    problem.options(1).nlpOpt.MaxFunEvals = 1e6; % objective fn evals
    problem.options(1).nlpOpt.MaxIter = 250; % fmincon iterations
    problem.options(1).hermiteSimpson.nSegment = 12; % # of segments between nodes
    problem.options(1).nlpOpt.UseParallel = false; % test

    %problem.options(2).method = 'hermiteSimpson';
    %problem.options(2).defaultAccuracy = 'high';
    %problem.options(2).nlpOpt.MaxFunEvals = 1e6;
    %problem.options(2).nlpOpt.MaxIter = 1e3;
    %problem.options(2).hermiteSimpson.nSegment = 60;
        % With nSegment = 60, it takes 128 seconds per iteration.

    % The art of optimization:
    % Initial Guess: Straight up flight
    % Push into low resolution solver -> 
    % Push low res solution into high res solver (verify)
    % debugging: fix it without attitude stuff then add it in
    % Initially set final pos, add complexity of turning later

g = problem.guess; f = problem.func.dynamics; k = 10;

% (4) guess vs bounds
bad = any(g.state < problem.bounds.state.low | g.state > problem.bounds.state.upp, 2);
disp(find(bad).')                                  % should be empty

% (1) vectorization: column k of a batch call must match a single call
dxAll = f(g.time, g.state, g.control);
dx1   = f(g.time(k), g.state(:,k), g.control(:,k));
disp(max(abs(dxAll(:,k) - dx1)))                   % should be ~0

% (3) purity: same call twice
disp(max(abs(f(g.time(k), g.state(:,k), g.control(:,k)) - dx1)))  % should be 0

% (2) sensitivity: perturb altitude and axial velocity
for s = [4 5]
    e = zeros(size(dx1)); e(s) = 1e-6;
    dxp = f(g.time(k), g.state(:,k)+e, g.control(:,k));
    fprintf('state %d -> max |d(dx)| = %.3g\n', s, max(abs(dxp - dx1)))   % should be nonzero
end
    % Call to OptimTraj
    addpath("+trajOptimizationPkg\OptimTraj\")
    soln = optimTraj(problem);

    disp('--- reached after optimTraj ---');
    xChk = interp1(check.time.', check.states.', soln(end).grid.time.').';
    fprintf('exitFlag = %d, max |u| = %.3g deg, max altitude diff vs ode45 = %.2f m\n', ...
        soln(end).info.exitFlag, max(abs(soln(end).grid.control), [], 'all'), ...
        max(abs(soln(end).grid.state(4,:) - xChk(4,:))));
    qF = soln(end).grid.state(8:11, end);

    trajectoryResolution = 250; % number of interpolated points in nominal traj

    nominalTrajectory.time = linspace(soln(end).grid.time(1), soln(end).grid.time(end), trajectoryResolution);
    nominalTrajectory.states = soln(end).interp.state(nominalTrajectory.time);
    nominalTrajectory.controlInputs = soln(end).interp.control(nominalTrajectory.time);
    nominalTrajectory.actuatorState = rocketDynamics.actuatorModel.allocationMatrix * nominalTrajectory.controlInputs;

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


function controller = setupDecoupledLQR(rocketDynamics, useActuatorModel, nominalTrajectory, environment)
    % Compute Reference Attitudes
    dt = 0.01;  % or whatever your timestep is
    controlTime = 23;  % or whatever your total time is
    time = linspace(dt, controlTime, controlTime/dt);
    n = length(time);
    
	% --- Define Trajectories in Degrees ---
	roll_deg  = zeros(1, n);
	pitch_deg = zeros(1, n);
	yaw_deg   = zeros(1, n);
	
	% Apply the step at 6 seconds
	% Logical mask for time >= 6 seconds
	mask_step1 = (time >= 2.0);
    mask_step2 = (time >= 9.0);
	
	% Set pitch to 12 degrees for all points where time >= 6
	yaw_deg(mask_step1) = 20.0;
    yaw_deg(mask_step2) = 0.0;

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
    tIgnition = 1;
    breakPoints = controllerPkg.BreakPoints(nominalTrajLog, numBreakPoints, cumVelocityInterval, tIgnition);

    % Get controller
	Q2 = diag([1 1]);       
	R2 = 20;   	
	Q3 = diag([1 1 1]);
	R3 = 2; 
    
    %groundAltitude = elevation(33.7490, -84.3880);
    groundAltitude = 0;

    [T, a, P, rho] = environment.getAtmosphere(groundAltitude);
    wind = environment.getWindVector(groundAltitude);
    envConditions = flightPkg.AtmosphericConditions(rho, a, P, T, wind.');

    reduceLTI = controllerPkg.ReduceLTI(rocketDynamics, envConditions);

    controller = controllerPkg.DecoupledLQR(breakPoints, referenceAttitudes, Q2, Q3, R2, R3, reduceLTI, useActuatorModel);
    controller.populateBreakPoints();
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
    % End conditions (whichever comes first)
    flightParams.totalTime = 150;  % max simulation length [s]
    flightParams.endState = flightPkg.FlightState.LANDED; % State to stop simulation

    % Flight state parameters
    flightParams.launchRailLength = 5.0;  % launch rail length [m]

    flightParams.dt = 0.01;  % time step [s]

    % Launch Location (Atlanta, GA)
    flightParams.launchLat = 33.7490; 
    flightParams.launchLon = -84.3880;

    % Initial conditions
    mass0 = rocketDynamics.massWet;
    pos0 = [0, 0, 0]; % pos0 is added to the default altitude given coordinates
    vel0 = [1, 0, 0];
	initAngle = Utils.eulerToQuat([0, 0, 0]).'; % yaw, pitch, roll
	quatRot = Utils.eulerToQuat([0, -90, 0]).';
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

    % Run Preflight Attitude Estimation
    % quat0 = navigation.preFlightAttitudeEstimation(state0, zeros(14, 1), 1, 1, eye(3), zeros(3, 1));
    % state0 = [mass0, pos0, vel0, quat0, omega0].';
    % flightParams.x0 = state0;


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

    % Extra chute (deploys at altitude)
    % extraDiameter = 1.54; % m
    % extraMass = 0.34; % kg
    % extraCD = 0.5;
    % extraDeployAlt = 100; % m
    % extra = rocketPkg.ParachuteDynamics("Extra Parachute", extraDiameter, extraMass, extraCD, extraDeployAlt);

    % Link parachutes: drogue -> main
    drogue.setNext(main);

    firstParachute = drogue;
end

function uncertaintyObj = setupUncertaintyParams(rocketDynamics, environment, flight)
    import analysisPkg.DistributionType
    
    uncertaintyObj = analysisPkg.Uncertainty();

    % Default uncertainty percentage (3-sigma as percentage of nominal)
    percentUncertainty = 0.1; % 10%

    % Aerodynamic scale factors
    caScaleFactor3Sigma = getThreeSigmaDeviation(rocketDynamics.body.scaleFactor, percentUncertainty);
    uncertaintyObj.addVariable('caScaleFactor', rocketDynamics.body.scaleFactor, caScaleFactor3Sigma, DistributionType.UNIFORM);

    cmScaleFactor3Sigma = getThreeSigmaDeviation(rocketDynamics.body.momentScaleFactor, percentUncertainty);
    uncertaintyObj.addVariable('cmScaleFactor', rocketDynamics.body.momentScaleFactor, cmScaleFactor3Sigma, DistributionType.UNIFORM);

    cnScaleFactor3Sigma = getThreeSigmaDeviation(rocketDynamics.body.scaleFactor, percentUncertainty);
    uncertaintyObj.addVariable('cnScaleFactor', rocketDynamics.body.scaleFactor, cnScaleFactor3Sigma, DistributionType.UNIFORM);

    currentParachute = rocketDynamics.activeParachute;

    while ~isempty(currentParachute)
        % Variable based on parachute name: parachuteCd_name 
        variableName = ['parachuteCd_' char(currentParachute.name)];
        
        parachuteCd3Sigma = getThreeSigmaDeviation(currentParachute.dragCoefficient, percentUncertainty);
        uncertaintyObj.addVariable(variableName, currentParachute.dragCoefficient, parachuteCd3Sigma, DistributionType.GAUSSIAN);
            
        % Move to next parachute
        currentParachute = currentParachute.nextParachute;
    end

    % Mass distribution
    wetMass3Sigma = getThreeSigmaDeviation(rocketDynamics.massWet, percentUncertainty);
    uncertaintyObj.addVariable('wetMass', rocketDynamics.massWet, wetMass3Sigma, DistributionType.GAUSSIAN);

    cgXWet3Sigma = getThreeSigmaDeviation(rocketDynamics.cgWet, percentUncertainty);
    uncertaintyObj.addVariable('cgXWet', rocketDynamics.cgWet, cgXWet3Sigma, DistributionType.GAUSSIAN);

    cgXDry3Sigma = getThreeSigmaDeviation(rocketDynamics.cgDry, percentUncertainty);
    uncertaintyObj.addVariable('cgXDry', rocketDynamics.cgDry, cgXDry3Sigma, DistributionType.GAUSSIAN);

    longitudinalMoiWet3Sigma = getThreeSigmaDeviation(rocketDynamics.moiWet(2,2), percentUncertainty);
    uncertaintyObj.addVariable('longitudinalMoiWet', rocketDynamics.moiWet(2,2), longitudinalMoiWet3Sigma, DistributionType.GAUSSIAN);

    rotationalMoiWet3Sigma = getThreeSigmaDeviation(rocketDynamics.moiWet(1,1), percentUncertainty);
    uncertaintyObj.addVariable('rotationalMoiWet', rocketDynamics.moiWet(1,1), rotationalMoiWet3Sigma, DistributionType.GAUSSIAN);

    longitudinalMoiDry3Sigma = getThreeSigmaDeviation(rocketDynamics.moiDry(2,2), percentUncertainty);
    uncertaintyObj.addVariable('longitudinalMoiDry', rocketDynamics.moiDry(2,2), longitudinalMoiDry3Sigma, DistributionType.GAUSSIAN);

    rotationalMoiDry3Sigma = getThreeSigmaDeviation(rocketDynamics.moiDry(1,1), percentUncertainty);
    uncertaintyObj.addVariable('rotationalMoiDry', rocketDynamics.moiDry(1,1), rotationalMoiDry3Sigma, DistributionType.GAUSSIAN);

    % Motor parameters
    totalImpulse3Sigma = getThreeSigmaDeviation(rocketDynamics.totalImpulse, percentUncertainty);
    uncertaintyObj.addVariable('totalImpulse', rocketDynamics.totalImpulse, totalImpulse3Sigma, DistributionType.GAUSSIAN);

    burnTime3Sigma = getThreeSigmaDeviation(rocketDynamics.burnTime, percentUncertainty);
    uncertaintyObj.addVariable('burnTime', rocketDynamics.burnTime, burnTime3Sigma, DistributionType.GAUSSIAN);

    tIgnition3Sigma = getThreeSigmaDeviation(rocketDynamics.tIgnition, percentUncertainty);
    uncertaintyObj.addVariable('tIgnition', rocketDynamics.tIgnition, tIgnition3Sigma, DistributionType.GAUSSIAN);

    % Body geometry
    noseLength3Sigma = getThreeSigmaDeviation(rocketDynamics.body.bodyParams.noseLength, percentUncertainty);
    uncertaintyObj.addVariable('noseLength', rocketDynamics.body.bodyParams.noseLength, noseLength3Sigma, DistributionType.GAUSSIAN);

    bodyLength3Sigma = getThreeSigmaDeviation(rocketDynamics.body.bodyParams.bodyLength, percentUncertainty);
    uncertaintyObj.addVariable('bodyLength', rocketDynamics.body.bodyParams.bodyLength, bodyLength3Sigma, DistributionType.GAUSSIAN);

    rocketDiameter3Sigma = getThreeSigmaDeviation(rocketDynamics.body.bodyParams.rocketDiameter, percentUncertainty);
    uncertaintyObj.addVariable('rocketDiameter', rocketDynamics.body.bodyParams.rocketDiameter, rocketDiameter3Sigma, DistributionType.GAUSSIAN);

    % Actuator parameters
    maxDeflection3Sigma = getThreeSigmaDeviation(rocketDynamics.actuatorModel.limits.maxDeflection, percentUncertainty);
    uncertaintyObj.addVariable('maxDeflection', rocketDynamics.actuatorModel.limits.maxDeflection, maxDeflection3Sigma, DistributionType.GAUSSIAN);

    maxRate3Sigma = getThreeSigmaDeviation(rocketDynamics.actuatorModel.limits.maxRate, percentUncertainty);
    uncertaintyObj.addVariable('maxRate', rocketDynamics.actuatorModel.limits.maxRate, maxRate3Sigma, DistributionType.GAUSSIAN);

    timeConstant3Sigma = getThreeSigmaDeviation(rocketDynamics.actuatorModel.timeConstant, percentUncertainty);
    uncertaintyObj.addVariable('timeConstant', rocketDynamics.actuatorModel.timeConstant, timeConstant3Sigma, DistributionType.GAUSSIAN);

    % Rail parameters
    railLength3Sigma = getThreeSigmaDeviation(flight.flightParams.launchRailLength, percentUncertainty);
    uncertaintyObj.addVariable('railLength', flight.flightParams.launchRailLength, railLength3Sigma, DistributionType.GAUSSIAN);

    railOrientation3Sigma = deg2rad(5);  % 5 degree uncertainty
    uncertaintyObj.addVariable('railOrientation', 0, railOrientation3Sigma, DistributionType.GAUSSIAN);

    % Wind
    U03sigma = getThreeSigmaDeviation(environment.U0, percentUncertainty);
    uncertaintyObj.addVariable('U0', environment.U0, U03sigma, DistributionType.GAUSSIAN)

    I03sigma = getThreeSigmaDeviation(environment.I0, percentUncertainty);
    uncertaintyObj.addVariable('I0', environment.I0, I03sigma, DistributionType.GAUSSIAN)
    
    theta03sigma = getThreeSigmaDeviation(environment.theta0, percentUncertainty);
    uncertaintyObj.addVariable('theta0', environment.theta0, theta03sigma, DistributionType.GAUSSIAN)

    % Navigation / Sensor parameters
    sigmaA3Sigma = getThreeSigmaDeviation(flight.navigation.sigmaA, percentUncertainty);
    uncertaintyObj.addVariable('sigmaA', flight.navigation.sigmaA, sigmaA3Sigma, DistributionType.GAUSSIAN);

    navPscaleNominal = flight.navigation.P(1,1);
    navPscale3Sigma = getThreeSigmaDeviation(navPscaleNominal, percentUncertainty);
    uncertaintyObj.addVariable('navPscale', navPscaleNominal, navPscale3Sigma, DistributionType.GAUSSIAN);

    gpsVerticalAcc3Sigma = getThreeSigmaDeviation(flight.navigation.gps.verticalAccuracy, percentUncertainty);
    uncertaintyObj.addVariable('gpsVerticalAccuracy', flight.navigation.gps.verticalAccuracy, gpsVerticalAcc3Sigma, DistributionType.GAUSSIAN);

    gpsHorizontalAcc3Sigma = getThreeSigmaDeviation(flight.navigation.gps.horizontalAccuracy, percentUncertainty);
    uncertaintyObj.addVariable('gpsHorizontalAccuracy',flight.navigation.gps.horizontalAccuracy, gpsHorizontalAcc3Sigma, DistributionType.GAUSSIAN);

    gpsOutputProb3Sigma = getThreeSigmaDeviation(flight.navigation.gps.outputProbability, percentUncertainty);
    uncertaintyObj.addVariable('gpsOutputProbability', flight.navigation.gps.outputProbability, gpsOutputProb3Sigma, DistributionType.GAUSSIAN);

    axisLabels = {'X','Y','Z'};
    for axisIdx = 1:3
        accVRWNominal = flight.navigation.accelerometer.VRW(axisIdx);
        accVRW3Sigma = getThreeSigmaDeviation(accVRWNominal, percentUncertainty);
        uncertaintyObj.addVariable(['accVRW' axisLabels{axisIdx}], accVRWNominal, accVRW3Sigma, DistributionType.GAUSSIAN);

        accBiasNominal = flight.navigation.accelerometer.bias(axisIdx);
        accBias3Sigma = getThreeSigmaDeviation(accBiasNominal, percentUncertainty);
        uncertaintyObj.addVariable(['accBias' axisLabels{axisIdx}], accBiasNominal, accBias3Sigma, DistributionType.GAUSSIAN);

        accSFNominal = flight.navigation.accelerometer.SF(axisIdx);
        accSF3Sigma = getThreeSigmaDeviation(accSFNominal, percentUncertainty);
        uncertaintyObj.addVariable(['accSF' axisLabels{axisIdx}], accSFNominal, accSF3Sigma, DistributionType.GAUSSIAN);

        gyroARWNominal = flight.navigation.gyroscope.ARW(axisIdx);
        gyroARW3Sigma = getThreeSigmaDeviation(gyroARWNominal, percentUncertainty);
        uncertaintyObj.addVariable(['gyroARW' axisLabels{axisIdx}], gyroARWNominal, gyroARW3Sigma, DistributionType.GAUSSIAN);

        gyroBiasNominal = flight.navigation.gyroscope.bias(axisIdx);
        gyroBias3Sigma = getThreeSigmaDeviation(gyroBiasNominal, percentUncertainty);
        uncertaintyObj.addVariable(['gyroBias' axisLabels{axisIdx}], gyroBiasNominal, gyroBias3Sigma, DistributionType.GAUSSIAN);
    end

end

function monteCarlo = setupMonteCarlo(uncertainty, flight, rocketDynamics, environment, logTemplate, numRuns)
    % Setup Monte Carlo simulation
    arguments
        uncertainty analysisPkg.Uncertainty
        flight flightPkg.Flight
        rocketDynamics rocketPkg.RocketDynamics
        environment flightPkg.Environment
        logTemplate analysisPkg.Log
        numRuns (1,1) double {mustBePositive, mustBeInteger}
    end

    monteCarlo = analysisPkg.MonteCarlo(uncertainty, flight, rocketDynamics, environment, logTemplate, numRuns);
end

function deviation = getThreeSigmaDeviation(nominalValue, percentUncertainty)
    % Helper function to calculate 3-sigma deviation from percentage
    arguments
        nominalValue {mustBeNumeric}
        percentUncertainty (1,1) double {mustBePositive}
    end

    deviation = abs(nominalValue) * percentUncertainty;
end



