%% Jet Vanes MasterFile Using GNCSIM 2.0 - NOT UPDATED
clear; clc; close all;

%% Instantiate Classes

% Setup Environment
environment = setupEnvironment();

% Setup CFD Aerodynamics
aerodynamics = setupAerodynamics();

% Setup Controller
% controller = setupController();
controller = -1;

% Setup BreakPoints
% breakPoints = setupBreakPoints(controller);
breakPoints = -1;

% Setup Navigation
navigation = setupNavigation();

% Rocket Dynamics
rocketDynamics = setupRocket(controller, aerodynamics); % setupRocket(controller, navigation, aerodynamics);

% Setup Flight Simulation
[flight, log] = setupFlight(rocketDynamics, environment, breakPoints, navigation); % added nav obj

%% Run Flight
log = flight.runSim(log);
log.trim();

%% Analysis
% Enforce custom time bounds
analysis = analysisPkg.Analysis(log, 0, 30);
analysis.displayPlots = true;

% Default filename 'Flight_Analysis_Report.pdf' in current directory
analysis.generatePdfFile();


%% Setup Functions

function rocketDynamics = setupRocket(controller, aerodynamics) % setupRocket(controller, navigation, aerodynamics);
    % Vehicle Properties
    rocketParams.longitudinalMoiWet = 24.655278;  % [kg * m^2]
    rocketParams.rotationalMoiWet = 0.209731;  % [kg * m^2]
	rocketParams.cgXWet = 2.419762;  % [m]
	rocketParams.massWet = 40.912994; % [kg]

	rocketParams.longitudinalMoiDry = 22.301315;  % [kg * m^2]
    rocketParams.rotationalMoiDry = 0.201846;  % [kg * m^2]
    rocketParams.cgXDry = 2.314814;  % [m]
	rocketParams.massDry = 32.619994; % [kg]

    % NOT CORRECT FOR JVR
    % % Create Body
    % bodyParams.bodyLength = 3.4;          % [m]
    % bodyParams.rocketDiameter = 0.203;    % [m]
    % 
    % % Create Body
    % bodyParams.noseLength = 0.61;  % [m]
    % bodyParams.bodyLength = 2.03;  % This does not include nose code [m]
    % bodyParams.rocketDiameter = 0.157;  % [m]
    % totalLength = bodyParams.bodyLength + bodyParams.noseLength;
	% bodyParams.translation = [0, 0, 0];
    % 
    % % Create Fins
    % finParams.numFins = 4;  % []
    % finParams.rootChord = 0.203;  % [m]
    % finParams.tipChord = 0.17;  % [m]
    % finParams.finSpan = 0.144;  % [m]
	% finParams.sweepDeg = 15.0;  % [deg]
    % 
    % flightConParams.machNumbers = [0.2, 0.4, 0.6];        % Array of Mach numbers to test
    % flightConParams.altitudes = [0, 5000, 10000];       % Array of altitudes in feet
    % flightConParams.alphasDeg   = [-10, -8, -6, -4, -2, 0, 2, 4, 6, 8, 10]; % Array of angles of attack in degrees
    % 
    % % get the distance along body
    % xFinRef = totalLength - finParams.rootChord;  % [m]
    % 
	% quarterChord = finParams.rootChord / 4;
	% bodyParams.xWapprox = 0;
	% bodyParams.cgXWet = rocketParams.cgXWet;
    % 
    % % set location of where to put fins
    % finParams.translation = [xFinRef, bodyParams.rocketDiameter / 2, 0];  % [m]
	% %finParams.translation = [xFinRe, bodyParams.rocketDiameter / 2, 0];  % [m]

    % Motor params
    motorParams.burnTime = 13.1; % s
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
    motorParams.totalImpulse = 14583; % N-sec

    % Create rocket dynamics object
    body = rocketPkg.RocketBody(bodyParams, flightConParams);
    fins = rocketPkg.RocketFins(finParams, flightConParams);
    
    rocketDynamics = rocketPkg.RocketDynamics(body, fins, rocketParams, motorParams, controller, aerodynamics); % rocketPkg.RocketDynamics(body, rocketFins, rocketParams, motorParams, controller, navigation, aerodynamics);
end

% Need to set up Flight to use environment()
function environmentObj = setupEnvironment()
    % Assuming Location of GRITS Launch Site; 31.2732° N, 83.3538° W
    latitude = 31.2732; % deg
    longitude = 83.3538; % deg
    elevation = 75; % m

    % Use default location? (if not, then python is needed to generate the new location data)
    useDefault = true;

    location = [latitude longitude elevation];
    environmentObj = flightPkg.Environment();
    environmentObj.setMaxAltitude(10000); % m
    environmentObj.setVmMean(0.01); % m/s
    environmentObj.setVm3Sigma(0.01); % m/s
end

% not working yet
function [aerodynamics] = setupAerodynamics()
    aerodynamics = rocketPkg.Aerodynamics();
    syms u v w rho CG p r
    vars = [u v w rho CG p r];
    
    fAxBSym = (1/0.225)*(((0.64181 .* ((v/u)*(180/pi)) .* (0.64181 ./ (3.59306 + sqrt(1 - ((3.281*(sqrt(u^2 + v^2 + w^2)))./1097.40).^2)))) .* 0.5 .* (rho*0.00194) .* (3.281*(sqrt(u^2 + v^2 + w^2))).^2 .* 0.9375 + 0.5*(rho*0.00194)*(3.281*(sqrt(u^2 + v^2 + w^2)))^2*0.9375 * (2.61799)/(1+sqrt(1+0.797628*(1-((3.281*(sqrt(u^2 + v^2 + w^2)))/1116.45)^2))) * asin(r*(10.3264-(3.281*CG))/(3.281*(sqrt(u^2 + v^2 + w^2)))))*sin(v/u) + ((0.00253 * ((v/u)*(180/pi)).^2 + 0.13932) .* 0.5 .* (rho*0.00194) .* (3.281*(sqrt(u^2 + v^2 + w^2))).^2 .* 0.9375)*-1*cos(v/u));
    fAyBSym = (1/0.225)*(((0.64181 .* ((v/u)*(180/pi)) .* (0.64181 ./ (3.59306 + sqrt(1 - ((3.281*(sqrt(u^2 + v^2 + w^2)))./1097.40).^2)))) .* 0.5 .* (rho*0.00194) .* (3.281*(sqrt(u^2 + v^2 + w^2))).^2 .* 0.9375 + 0.5*(rho*0.00194)*(3.281*(sqrt(u^2 + v^2 + w^2)))^2*0.9375 * (2.61799)/(1+sqrt(1+0.797628*(1-((3.281*(sqrt(u^2 + v^2 + w^2)))/1116.45)^2))) * asin(r*(10.3264-(3.281*CG))/(3.281*(sqrt(u^2 + v^2 + w^2)))))*-1*cos(v/u) + ((0.00253 * ((v/u)*(180/pi)).^2 + 0.13932) .* 0.5 .* (rho*0.00194) .* (3.281*(sqrt(u^2 + v^2 + w^2))).^2 .* 0.9375)*-1*sin(v/u));
    fAzBSym = 0;
    mAxBSym = 1.36*((((4.*((2.*pi)./sqrt(1-((3.281*sqrt(u^2 + v^2 + w^2))./1116.4).^2)).*p)./(0.4688.*(3.281*sqrt(u^2 + v^2 + w^2)))).*0.1897).*0.5.*(rho*0.00194).*(3.281*sqrt(u^2 + v^2 + w^2)).^2.*0.4688);
    mAyBSym = 0;
    mAzBSym = 1.36*((((v/u)*(180/pi)) .* ((4.12421 ./ (27.81553 + 16.83430 .* sqrt(1 - ((3.281*sqrt(u^2 + v^2 + w^2))./1097.40).^2))) + -0.00014 .* ((v/u)*(180/pi)) + 0.01273)) * 0.5 * (rho*0.00194) * (3.281*sqrt(u^2 + v^2 + w^2)).^2 * 0.9375 + 0.5*(rho*0.00194)*(3.281*sqrt(u^2 + v^2 + w^2))^2*0.9375 * (2.61799)/(1+sqrt(1+0.797628*(1-((3.281*sqrt(u^2 + v^2 + w^2))/1116.45)^2))) * asin(r*(10.3264-(3.281*CG))/(3.281*sqrt(u^2 + v^2 + w^2))) * (10.3264-(3.281*CG)));
    
    fAxBFunc = matlabFunction(fAxBSym, vars=vars);
    fAyBFunc = matlabFunction(fAyBSym, vars=vars);
    fAzBFunc = matlabFunction(fAzBSym, vars=vars);
    mAxBFunc = matlabFunction(mAxBSym, vars=vars);
    mAyBFunc = matlabFunction(mAyBSym, vars=vars);
    mAzBFunc = matlabFunction(mAzBSym, vars=vars);

    aerodynamics.setFAxFunc(fAxBFunc);
    aerodynamics.setFAyFunc(fAyBFunc);
    aerodynamics.setFAzFunc(fAzBFunc);
    aerodynamics.setMAxFunc(mAxBFunc);
    aerodynamics.setMAyFunc(mAyBFunc);
    aerodynamics.setMAzFunc(mAzBFunc);

    aeroForcesMoments = [fAxBSym, fAyBSym, fAzBSym, mAxBSym, mAyBSym, mAzBSym];
end

function breakPoints = setupBreakPoints()
    % TODO
end 

function controller = setupController() % Where should I implement trimpoints?
    % State: [u, v, w, p, q, r, q0, q1, q2, q3]
    Q = diag([10, 10, 10, 1, 1, 1, 1, 1, 1, 1]);   % Not Sure
    R = diag([1, 1, 1, 1]);                        % Not Sure

    % Create Jet Vanes Actuator

    % should set the bias and r24 here, not through motor params
    jetVanesActuator = jvrFiles.JetVanesActuatorModel();
    jetVanesActuator.biasAng = [0; 0; 0; 0]; % [rad] Not Sure What Real Values Are
    jetVanesActuator.r24 = 0.0144;              % [m] Distance Between Vanes 2 and 4
    jetVanesActuator.location = 3.4;            % m (length of rocket)
    jetVanesActuator.qBa = [1, 0, 0, 0];       % unit quaternion

    % Create LQR Controller
    controller = controllerPkg.LQR(Q, R, jetVanesActuator);
end

function nav = setupNavigation()
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

    navParams.dt = .01;
    dt = navParams.dt;
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


function [flight, log] = setupFlight(rocketDynamics, environment, breakPoints, navigation)
    % Flight
    flightParams.totalTime = 50;  % length of simulation [s]
    flightParams.dt = 0.01;  % time step [s]

    mass0 = rocketDynamics.massWet;
    pos0 = [0, 0, 0];
    vel0 = [0, 0, 0];
    quat0 = Utils.eulerToQuat([0, -90, 0]).';
    omega0 = [0, 0, 0];
    state0 = [mass0, pos0, vel0, quat0, omega0].';
    flightParams.x0 = state0;

    breakPoints = -1;
    environment = -1;

    flight = flightPkg.Flight(rocketDynamics, environment, breakPoints, navigation, flightParams);

    % Define state names (14 states for 6DOF rocket)
    stateNames = {'mass', 'x', 'y', 'z', 'vx', 'vy', 'vz', ...
                  'q0', 'q1', 'q2', 'q3', 'wx', 'wy', 'wz'};
    
    % Define control input names
    controlInputNames = {'thrust', 'gimbalX', 'gimbalY', 'gimbalZ'};

    % Size of log (will increase if needed)
    maxSteps = flightParams.totalTime / flightParams.dt;
    
    % Initialize Log object
    log = analysisPkg.Log(stateNames, controlInputNames, maxSteps);
end




