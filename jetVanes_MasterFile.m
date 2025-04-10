%% Master GNCSIM Simulation File for Jet Vanes Rocket

clear; clc; close all;

%% Rocket Parameters

wet_mass = 40.912994; % kg
dry_mass = 32.619994;
initial_CG = 2.419762;
final_CG = 2.314814;

Doof = Rocket();
Doof.set_wetMass(wet_mass);
Doof.set_dryMass(dry_mass);
Doof.set_diameter(0.203); % rocket diameter (m)
Doof.set_radius(Doof.get_diameter()/2);
Doof.set_length(3.4); % length of rocket (m)
Doof.set_initialCG(initial_CG); % initial CG from tip of nosecone (m)
Doof.set_finalCG(final_CG); % final CG from tip of nosecone (m)

wetRollMOI = 0.209731;  % Ixx
wetPitchMOI = 24.655278;  % Iyy
wetYawMOI = 24.656148;  % Izz
dryRollMOI = 0.201846;
dryPitchMOI = 22.301315;
dryYawMOI = 22.301570;

burn_time = 13.1;

Doof.set_mainChuteCd(2.2);
Doof.set_mainChuteArea(7.2965877);
Doof.set_drogueChuteCd(1.55);
Doof.set_drogueChuteArea(0.291863508);

Doof.set_wetMOI([wetRollMOI 0 0; 0 wetPitchMOI 0; 0 0 wetYawMOI]);
Doof.set_dryMOI([dryRollMOI 0 0; 0 dryPitchMOI 0; 0 0 dryYawMOI]);
Doof.set_burnTime(burn_time);

tc = [0.0, 0.457288;
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
Doof.set_thrustCurve(tc);

% Note: Aero model for jet vanes rocket doesn't use a CP, moments are
% computed about CG directly

%% Aerodynamic Parameters
AeroObj = Aerodynamics();

syms u v w rho CG p r
Vars = [u v w rho CG p r];

FAx_b_sym = (1/0.225)*(((0.64181 .* ((v/u)*(180/pi)) .* (0.64181 ./ (3.59306 + sqrt(1 - ((3.281*(sqrt(u^2 + v^2 + w^2)))./1097.40).^2)))) .* 0.5 .* (rho*0.00194) .* (3.281*(sqrt(u^2 + v^2 + w^2))).^2 .* 0.9375 + 0.5*(rho*0.00194)*(3.281*(sqrt(u^2 + v^2 + w^2)))^2*0.9375 * (2.61799)/(1+sqrt(1+0.797628*(1-((3.281*(sqrt(u^2 + v^2 + w^2)))/1116.45)^2))) * asin(r*(10.3264-(3.281*CG))/(3.281*(sqrt(u^2 + v^2 + w^2)))))*sin(v/u) + ((0.00253 * ((v/u)*(180/pi)).^2 + 0.13932) .* 0.5 .* (rho*0.00194) .* (3.281*(sqrt(u^2 + v^2 + w^2))).^2 .* 0.9375)*-1*cos(v/u));
FAy_b_sym = (1/0.225)*(((0.64181 .* ((v/u)*(180/pi)) .* (0.64181 ./ (3.59306 + sqrt(1 - ((3.281*(sqrt(u^2 + v^2 + w^2)))./1097.40).^2)))) .* 0.5 .* (rho*0.00194) .* (3.281*(sqrt(u^2 + v^2 + w^2))).^2 .* 0.9375 + 0.5*(rho*0.00194)*(3.281*(sqrt(u^2 + v^2 + w^2)))^2*0.9375 * (2.61799)/(1+sqrt(1+0.797628*(1-((3.281*(sqrt(u^2 + v^2 + w^2)))/1116.45)^2))) * asin(r*(10.3264-(3.281*CG))/(3.281*(sqrt(u^2 + v^2 + w^2)))))*-1*cos(v/u) + ((0.00253 * ((v/u)*(180/pi)).^2 + 0.13932) .* 0.5 .* (rho*0.00194) .* (3.281*(sqrt(u^2 + v^2 + w^2))).^2 .* 0.9375)*-1*sin(v/u));
FAz_b_sym = 0;
MAx_b_sym = 1.36*((((4.*((2.*pi)./sqrt(1-((3.281*sqrt(u^2 + v^2 + w^2))./1116.4).^2)).*p)./(0.4688.*(3.281*sqrt(u^2 + v^2 + w^2)))).*0.1897).*0.5.*(rho*0.00194).*(3.281*sqrt(u^2 + v^2 + w^2)).^2.*0.4688);
MAy_b_sym = 0;
MAz_b_sym = 1.36*((((v/u)*(180/pi)) .* ((4.12421 ./ (27.81553 + 16.83430 .* sqrt(1 - ((3.281*sqrt(u^2 + v^2 + w^2))./1097.40).^2))) + -0.00014 .* ((v/u)*(180/pi)) + 0.01273)) * 0.5 * (rho*0.00194) * (3.281*sqrt(u^2 + v^2 + w^2)).^2 * 0.9375 + 0.5*(rho*0.00194)*(3.281*sqrt(u^2 + v^2 + w^2))^2*0.9375 * (2.61799)/(1+sqrt(1+0.797628*(1-((3.281*sqrt(u^2 + v^2 + w^2))/1116.45)^2))) * asin(r*(10.3264-(3.281*CG))/(3.281*sqrt(u^2 + v^2 + w^2))) * (10.3264-(3.281*CG)));

FAx_b_func = matlabFunction(FAx_b_sym, vars=Vars);
FAy_b_func = matlabFunction(FAy_b_sym, vars=Vars);
FAz_b_func = matlabFunction(FAz_b_sym, vars=Vars);
MAx_b_func = matlabFunction(MAx_b_sym, vars=Vars);
MAy_b_func = matlabFunction(MAy_b_sym, vars=Vars);
MAz_b_func = matlabFunction(MAz_b_sym, vars=Vars);

AeroObj.set_FAx_func(FAx_b_func);
AeroObj.set_FAy_func(FAy_b_func);
AeroObj.set_FAz_func(FAz_b_func);
AeroObj.set_MAx_func(MAx_b_func);
AeroObj.set_MAy_func(MAy_b_func);
AeroObj.set_MAz_func(MAz_b_func);

%% Environment Parameters

% Assuming Location of GRITS Launch Site; 31.2732° N, 83.3538° W
latitude = 31.2732; % deg
longitude = 83.3538; % deg
elevation = 75; % m

% Use default location? (if not, then python is needed to generate the new location data)
use_default = true;

location = [latitude longitude elevation];
EnvironmentObj = Environment();
EnvironmentObj.set_max_altitude(10000) % m
EnvironmentObj.set_vm_mean(0.01) % m/s
EnvironmentObj.set_vm_3sigma(0.01) % m/s


%% Flight Parameters

FlightObj = Flight();
FlightObj.set_tStep(0.005); % s
FlightObj.set_runClosedLoop(false);
FlightObj.set_calcXDots(true);
FlightObj.set_runLinearModel(false);
FlightObj.set_applyWind(false);
FlightObj.set_numStates(13);
FlightObj.set_numInputs(3);
FlightObj.set_simTime(50); % seconds
FlightObj.set_localSimTime(50); % linear model propagation time, jet vanes linear model only goes up to 12 seconds
FlightObj.set_tSpan()
FlightObj.set_odeSolverMaxTime(300) % 300s before killing the simulation
%flight.set_tSpan(0:flight.get_tStep():flight.get_simTime());
FlightObj.set_controlsTime(12);

FlightObj.set_uSpan(zeros(length(FlightObj.get_tSpan()), FlightObj.get_numInputs()));

initial_state = [0; % px
                 0; % py
                 0; % pz
                 0.001; % u
                 0; % v
                 0; % w
                 1; % q0
                 0; % q1
                 0; % q2
                 0; % q3
                 0; % p
                 0; % q
                 0]; % r
FlightObj.set_initState(initial_state);

% At this point, should be able to run a single nonlinear flight
%flight.run_Flight()
%flight.plot_sim_dashboard()

%% Trim Point Parameters for A and B Matrices

sympref("FloatingPointOutput",true);
syms m g Ixx Iyy Izz rho CG t
syms u v w p q r q0 q1 q2 q3
syms udot vdot wdot pdot qdot rdot q1dot q2dot q3dot
syms u1 u2 u3 u4 % Thrust, roll moment, pitch moment, yaw moment

EOM_no_aero = [(1/m)*(m*g*(1-2*(q0^2+q1^2)) + 0.95*u4) + r*v - q*w - udot,
    -2*g*(q1*q2 + q0*q3) + p*w - r*u - vdot,
    -2*g*(q1*q3 - q0*q2) + q*u - p*v - wdot,
    (1/Ixx)*(u1 + q*r*(Iyy - Izz)) - pdot,
    (1/Iyy)*(u2 + p*r*(Izz - Ixx)) - qdot,
    (1/Izz)*(u3 + p*q*(Ixx - Iyy)) - rdot,
     0.5*(p*q0 + r*q2 - q*q3) - q1dot,
     0.5*(q*q0 - r*q1 + p*q3) - q2dot,
     0.5*(r*q0 + q*q1 - p*q2) - q3dot];

FAx = FAx_b_sym;
FAy = FAy_b_sym;
FAz = FAz_b_sym;
MAx = MAx_b_sym;
MAy = MAy_b_sym;
MAz = MAz_b_sym;

EOM_only_aero = [(1/m)*FAx,
                 (1/m)*FAy,
                 (1/m)*FAz,
                 (1/Ixx)*MAx,
                 (1/Iyy)*MAy,
                 (1/Izz)*MAz,
                 0,
                 0,
                 0]; 

CG_t = final_CG + (initial_CG - final_CG) * (burn_time - t) / burn_time;
Ixx_t = dryRollMOI + (wetRollMOI - dryRollMOI) * (burn_time - t) / burn_time;
Iyy_t = dryPitchMOI + (wetPitchMOI - dryPitchMOI) * (burn_time - t) / burn_time;
Izz_t = dryYawMOI + (wetYawMOI - dryYawMOI) * (burn_time - t) / burn_time;
m_t = dry_mass + (wet_mass - dry_mass) * (burn_time - t) / burn_time;

CG_t_handle = matlabFunction(CG_t);
rho_t_handle = @(t) 1.2;

%nonlinear_EOM = EOM_no_aero;
nonlinear_EOM = EOM_no_aero + EOM_only_aero;

F = nonlinear_EOM;
xdot = [udot, vdot, wdot, pdot, qdot, rdot, q1dot, q2dot, q3dot];
x = [u, v, w, p, q, r, q1, q2, q3];
u = [u1, u2, u3, u4];

% Find E, A', and B'

E = jacobian(F, xdot);
A_prime = jacobian(F, x);
B_prime = jacobian(F, u);

A_prime = subs(A_prime, [rho, g, Ixx, Iyy, Izz, CG, m], [1.2, 9.8, Ixx_t, Iyy_t, Izz_t, CG_t, m_t]);
B_prime = subs(B_prime, [rho, g, Ixx, Iyy, Izz, CG, m], [1.2, 9.8, Ixx_t, Iyy_t, Izz_t, CG_t, m_t]);

% Find A, B

A = -1*inv(E)*A_prime; % NOTE: The actual formula is -1*inv(E)*A_prime and same for B_prime, however, 
% inv(E) was returning a diagonal matrix of negative infinities. In other
% sections, it was not. This was weird, and because E is a diagonal with
% all -1's, inv(E) = E, so it is just left as -1*E*A_prime
B = -1*inv(E)*B_prime;

% Then delta_xdot = A*delta_x + B*delta_u
% xdot_0, x_0, u_0 <--- trim point
% xdot, x, u <--- current rocket state
% delta_xdot = xdot - dxot_0, delta_x = x - x_0, delta_u = u - u0

compute_A_fn = matlabFunction(A);

compute_A = @(u, v, w, p, q, r, q0, q1, q2, q3, t)compute_A_fn(p,q,q0,q1,q2,q3,r,t,u,v,w);
compute_B = matlabFunction(B);

%% Trim Point Generation
% csv generated from open rocket
flight_path = 'Flight Profile\JVR_sim.csv';

% Create a TrimPoints object by initializing it with the specified flight data file
TrimPointsObj = TrimPoints(flight_path);

% Find the time of the "LIFTOFF" event in the flight data
liftoff_time = TrimPointsObj.find_event_time("LIFTOFF");
%liftoff_time = 0.18;

% Find the time of the burnout
burnout_time = burn_time + liftoff_time;

trim_switching_period = 1;
    
time_arr = liftoff_time:trim_switching_period:burnout_time - trim_switching_period;

FlightObj.set_runClosedLoop(false) % Needed to optimize trimpoints

generate_trim_points = true;
if generate_trim_points
    % Generate trim batches for each time point in time_arr
    TrimPointsObj.generate_trim_batches(time_arr);
    
    % Define the number of trim points to generate per time instance for 'u' variations
    num_trim_points_per_time = 11;
    
    % Add 'u' (velocity) variations to each trim batch based on the specified parameters
    % This creates different trim points by varying the 'u' component within the defined spacing
    % THIS FUNCTION SHOULD BE CHANGED TO INCLUDE AERO MOMENTS
    TrimPointsObj.add_u_variations(time_arr, num_trim_points_per_time, AeroObj, CG_t_handle, rho_t_handle);   
    
    turn_batch_index = 1:length(time_arr);  % Define the indices of batches to which custom variations will be added
    turn_variation_name = "turn";  % Define the name of the custom variation to add
    
    % Define the variables involved in the custom variation
    % In this case, the variation affects the quaternion components q0, q1, q2, and q3
    turn_variables = ["q0", "q1", "q2", "q3"];
    
    % Define the variation data matrix for the custom variation
    % Each row represents a different state for the variation
    %turn_data = [0.9990482, 0, 0, 0.0436194;  % 5 deg
    %             0.9961947, 0, 0, 0.0871557;  % 10 deg
    %             0.9914449, 0, 0, 0.1305262]; % 15 deg

    turn_data = [0.999762, 0, 0, 0.0218149;   % 2.5 deg
                 0.9990482, 0, 0, 0.0436194]; % 5 deg

    
    % include base state [0, 0, 0, 1] strait up flight
    turn_include_base = true;
    
    % This will create additional trim points by varying the specified quaternion components
    TrimPointsObj.add_custom_variations(turn_batch_index, turn_variation_name, turn_variables, turn_data, turn_include_base);
    
    % This consolidates all trim points into the obj.trim_points property for
    % further use AND convert all trim points to trimmed points.
    TrimPointsObj.generate_trim_points(Doof, EnvironmentObj, FlightObj, AeroObj);
else
    load("trimObj.mat", "TrimPointsObj");
end

% straight up Q and R
straight_up_Q_arr = ones(9, 1);
straight_up_R_arr = ones(4, 1);

straight_up_Q_arr(1) = 0.01; % u
straight_up_Q_arr(2) = 0.01; % v
straight_up_Q_arr(3) = 0.01; % w
straight_up_Q_arr(4) = 1000; % p
straight_up_Q_arr(5) = 0.01; % q
straight_up_Q_arr(6) = 1000; % r
straight_up_Q_arr(7) = 1000; % q1
straight_up_Q_arr(8) = 0.01; % q2
straight_up_Q_arr(9) = 100000; % q3

straight_up_R_arr(1) = 0.01; % M_roll
straight_up_R_arr(2) = 1000; % M_pitch
straight_up_R_arr(3) = 0.001; % M_yaw
straight_up_R_arr(4) = 100; % Thrust

straight_up_Q = diag(1*straight_up_Q_arr);
straight_up_R = diag(10*straight_up_R_arr);

% maneuver Q and R
% maneuver_Q_arr = ones(9, 1);
% maneuver_R_arr = ones(4, 1);
% 
% maneuver_Q_arr(1) = 0.01; % u
% maneuver_Q_arr(2) = 0.01; % v
% maneuver_Q_arr(3) = 0.01; % w
% maneuver_Q_arr(4) = 1000; % p
% maneuver_Q_arr(5) = 0.01; % q
% maneuver_Q_arr(6) = 1000; % r
% maneuver_Q_arr(7) = 1000; % q1
% maneuver_Q_arr(8) = 0.01; % q2
% maneuver_Q_arr(9) = 10000; % q3
% 
% maneuver_R_arr(1) = 0.01; % M_roll
% maneuver_R_arr(2) = 1000; % M_pitch
% maneuver_R_arr(3) = 0.001; % M_yaw
% maneuver_R_arr(4) = 1000; % Thrust

maneuver_Q = straight_up_Q;
maneuver_R = straight_up_R;

% Generate K matrix for all trim points given Q and R matrixes
TrimPointsObj.generate_K(straight_up_Q, straight_up_R, maneuver_Q, maneuver_R, compute_A, compute_B);

save_h_data = true;
if save_h_data
    TrimPointsObj.export_to_ref_h('data/h_data.h')
end
% Visualize 'u' variations
%TrimPointsObj.draw_u_trim_points(0, burnout_time)

%A_current_time_step = compute_A(2.43,1,10,10,9.81,33,0.02,-0.04,0.962,0,0,0.0872,0.07,0.8,150,0.4,0.02)
%B_current_time_step = compute_B(1,10,10,33)
% The above are function handles that can compute A, B given a current
% rocket state


%% Setting Reference States
% Define time array and variations for the test

variations_cell_arr = {
    {time_arr(5), {'r', 0.1745}, {'q0', 0.9990482}, {'q3', 0.0436194}};   % ~4th second
    {time_arr(6), {'r', 0.1745}, {'q0', 0.9990482}, {'q3', 0.0436194}};   % ~5th second
    {time_arr(7), {'q0', 0.9990482}, {'q3', 0.0436194}};                  % ~6th second
    {time_arr(8), {'q0', 0.9990482}, {'q3', 0.0436194}};                  % ~7th second
    {time_arr(9), {'r', -0.1745}, {'q0', 0.9990482}, {'q3', 0.0436194}}; % ~8th second
    {time_arr(9), {'r', -0.1745}}                                   % 9th second
};

% Call the set_reference_states method
TrimPointsObj.set_reference_states(time_arr, variations_cell_arr);

%% RunFlight
FlightObj.set_runClosedLoop(false); % Set to true to simulate controlled flight
FlightObj.set_applyWind(false);
log = FlightObj.runFlight(Doof, EnvironmentObj, AeroObj, TrimPointsObj);
FlightObj.plot_sim_dashboard(log, Doof, EnvironmentObj);

%% Set Up MonteCarlo and get Latin Hypercube Samples
monteCarlo = MonteCarlo();

% Set hyperparameters
monteCarlo.set_runGaussianLHS(true);
monteCarlo.set_numRuns(100);
monteCarlo.set_odeSolverMaxTime(150); % kills ode after x seconds

% Set 3sigma values 
monteCarlo.set_wetMass_3sigma(4.0912994);
monteCarlo.set_dryMass_3sigma(3.2619994);

monteCarlo.set_wetRollMOI_3sigma(0.0209731);
monteCarlo.set_wetPitchMOI_3sigma(2.4655278);
monteCarlo.set_wetYawMOI_3sigma(2.4656148);
monteCarlo.set_dryRollMOI_3sigma(0.0201846);
monteCarlo.set_dryPitchMOI_3sigma(2.2301315);
monteCarlo.set_dryYawMOI_3sigma(2.2301570);

monteCarlo.set_initialCG_3sigma(0.2419762);
monteCarlo.set_finalCG_3sigma(0.2314814);

monteCarlo.set_burnTime_3sigma(1.31);
monteCarlo.set_thrustCurve_3sigma(100);

EnvironmentObj.set_vm_mean(0);
EnvironmentObj.set_vm_3sigma(15);

% Saving parameters in CSV for future use
monteCarlo.saveToCSV('monteCarloParameters.csv')

% Setting up FlightObj
FlightObj.set_runClosedLoop(true);
FlightObj.set_applyWind(true);

% Genereate Latin Hypercube Samples 
monteCarlo.generateLhsSamples(Doof);

% Splits LHS in x parts and saves to folder
% splittedLHS = monteCarlo.splitLHS(10, 'LHS_splits'); 

%% MonteCarlo Run
logMonteCarlo = monteCarlo.runMonteCarlo(FlightObj, Doof, EnvironmentObj, AeroObj, TrimPointsObj);

%% MonteCarlo Pass/Fail Evaluation
monteCarlo.set_evaluate_checkTermination(true);
monteCarlo.set_evaluate_checkRoll(true);
monteCarlo.set_evaluate_checkYaw(true);
monteCarlo.set_evaluate_checkAoA(true);

% Each sub-cell element has two entries:
%   - A time range (vector)
%   - A two-element vector for the acceptable boundaries.
% Example: {{5:6, [5, 10]}, {11:12, [-2, 2]}} = yaw between 5 and 10
% degrees at seconds 5 and 6, and between -2 and 2 degrees at seconds 11 and
% 12.
monteCarlo.set_evaluate_yawRange({{5:6, [5, 15]}, {12, [-2, 2]}});
monteCarlo.set_evaluate_rollRange({{1:12, [-2, 2]}});
monteCarlo.set_evaluate_AoARange({{1:12, [-10, 10]}});

combinedLogMonteCarlo = monteCarlo.evaluateMonteCarlo(combinedLogMonteCarlo);

%% Plotting MonteCarlo Analysis
monteCarlo = MonteCarlo();
monteCarlo.plotMonteCarlo(combinedLogMonteCarlo)

%% Running SplitLHS
monteCarlo = MonteCarlo();
monteCarlo.set_odeSolverMaxTime(150); % kills ode after x seconds
EnvironmentObj.set_vm_mean(0);
EnvironmentObj.set_vm_3sigma(15);

% Setting up FlightObj
FlightObj.set_runClosedLoop(true);
FlightObj.set_applyWind(true);


monteCarlo.set_lhsSamples(lhsChunk);
logMonteCarlo = monteCarlo.runMonteCarlo(FlightObj, Doof, EnvironmentObj, AeroObj, TrimPointsObj);

%% Combining partial MonteCarlo runs

combinedLogMonteCarlo = [
    logMonteCarlo1';
    logMonteCarlo2';
    logMonteCarlo3';
    logMonteCarlo4';
    logMonteCarlo5';
    logMonteCarlo6';
    logMonteCarlo7';
    logMonteCarlo8';
    logMonteCarlo9';
    logMonteCarlo10'
];





