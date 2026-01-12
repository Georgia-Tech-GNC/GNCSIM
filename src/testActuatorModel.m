%% Test Actuator Model Forces and Moments
clear; clc; close all;

%% Setup Flight Conditions
flightConParams.machNumbers = [0.2, 0.4, 0.6];
flightConParams.altitudes = [0, 5000, 10000];
flightConParams.alphasDeg = [-10, -8, -6, -4, -2, 0, 2, 4, 6, 8, 10];

%% Create Dynamic Fins
finTabLength = 0.05;
bodyParams.noseLength = 0.61;
bodyParams.bodyLength = 2.2856;
bodyParams.rocketDiameter = 0.157;
totalLength = bodyParams.bodyLength + bodyParams.noseLength;

finParamsTabs.numFins = 4;
finParamsTabs.rootChord = finTabLength;
finParamsTabs.tipChord = finTabLength;
finParamsTabs.finSpan = 0.144;
finParamsTabs.sweepDeg = 0.0;
finParamsTabs.movable = true;
finParamsTabs.name = 'DYNAMIC_FIN';
finParamsTabs.translation = [totalLength-finTabLength, bodyParams.rocketDiameter / 2, 0];

rocketFinTabs = rocketPkg.RocketFins(finParamsTabs, flightConParams);

%% Create Actuator Model
allocationMatrix = [0, -1, 0, -1;
                    -1, 0, 1, 0;
                    0, 1, 0, -1].';

timeConstant = 0.05; % [s]
actuatorModel = theOncelerFiles.TheOncelerActuatorModel(rocketFinTabs, timeConstant, allocationMatrix);

%% Build the aerodynamic data
fprintf('Building aerodynamic data...\n');
actuatorModel.computeBuildup();
fprintf('Build complete.\n\n');

%% Define Nominal State
% State vector: [mass, x, y, z, vx, vy, vz, q0, q1, q2, q3, wx, wy, wz]
mass = 29.04; % kg
pos = [0, 0, 1000]; % [m]
vel = [100, 0, 0]; % [m/s] - 100 m/s in x direction
quat = Utils.eulerToQuat([0, -90, 0]).'; % nose pointing in +x direction
omega = [0, 0, 0]; % [rad/s]
state = [mass, pos, vel, quat, omega].';

fprintf('Nominal State:\n');
fprintf('  Velocity: [%.1f, %.1f, %.1f] m/s\n', vel);
fprintf('  Altitude: %.1f m\n', pos(3));
fprintf('  Speed: %.1f m/s (Mach %.2f)\n', norm(vel), norm(vel)/340);
fprintf('\n');

%% Set Actuator State (5 degrees on all fins)
actuatorState = [5, 5, 5, 5].'; % degrees
fprintf('Actuator State: [%.1f, %.1f, %.1f, %.1f] deg\n\n', actuatorState);

%% Compute Forces and Moments
currentCg = 1.98; % [m] - x-position of CG
disturbances.wind = [0, 0, 0].';

[fB, mB] = actuatorModel.getForcesMoments(state, actuatorState, currentCg, disturbances);

%% Display Results
fprintf('=== Forces and Moments from Actuator ===\n');
fprintf('Forces in Body Frame [N]:\n');
fprintf('  Fx: %10.4f N\n', fB(1));
fprintf('  Fy: %10.4f N\n', fB(2));
fprintf('  Fz: %10.4f N\n', fB(3));
fprintf('  |F|: %10.4f N\n', norm(fB));
fprintf('\n');

fprintf('Moments in Body Frame [Nm]:\n');
fprintf('  Mx (Roll):  %10.4f Nm\n', mB(1));
fprintf('  My (Pitch): %10.4f Nm\n', mB(2));
fprintf('  Mz (Yaw):   %10.4f Nm\n', mB(3));
fprintf('  |M|: %10.4f Nm\n', norm(mB));
fprintf('\n');

%% Test with zero deflection for comparison
actuatorStateZero = [0, 0, 0, 0].';
[fB_zero, mB_zero] = actuatorModel.getForcesMoments(state, actuatorStateZero, currentCg, disturbances);

fprintf('=== Forces and Moments with Zero Deflection ===\n');
fprintf('Forces in Body Frame [N]:\n');
fprintf('  Fx: %10.4f N\n', fB_zero(1));
fprintf('  Fy: %10.4f N\n', fB_zero(2));
fprintf('  Fz: %10.4f N\n', fB_zero(3));
fprintf('  |F|: %10.4f N\n', norm(fB_zero));
fprintf('\n');

fprintf('Moments in Body Frame [Nm]:\n');
fprintf('  Mx (Roll):  %10.4f Nm\n', mB_zero(1));
fprintf('  My (Pitch): %10.4f Nm\n', mB_zero(2));
fprintf('  Mz (Yaw):   %10.4f Nm\n', mB_zero(3));
fprintf('  |M|: %10.4f Nm\n', norm(mB_zero));
fprintf('\n');

%% Test Actuator Dynamics
fprintf('=== Testing Actuator Dynamics ===\n');
controlInput = [0, 5, 0].'; % 5 degrees pitch command
currentActuatorState = [0, 0, 0, 0].'; % starting from zero

actuatorStateDot = actuatorModel.actuatorDynamics(controlInput, currentActuatorState);

fprintf('Control Input: [Roll=%.1f, Pitch=%.1f, Yaw=%.1f] deg\n', controlInput);
fprintf('Current Actuator State: [%.1f, %.1f, %.1f, %.1f] deg\n', currentActuatorState);
fprintf('Actuator State Derivative: [%.1f, %.1f, %.1f, %.1f] deg/s\n', actuatorStateDot);
fprintf('Time constant: %.2f s\n', timeConstant);
fprintf('\n');

% Calculate commanded state
actuatorStateCmd = allocationMatrix * controlInput;
fprintf('Commanded Actuator State (from allocation): [%.1f, %.1f, %.1f, %.1f] deg\n', actuatorStateCmd);
