%% Master GNCSIM Simulation File for Jet Vanes Rocket

clear; clc; close all;

%% Rocket Parameters

wet_mass = 38.87;
dry_mass = 30.96;
initial_CG = 2.433;
final_CG = 2.34;

Doof = Rocket();
Doof.set_wetMass(wet_mass);
Doof.set_dryMass(dry_mass);
Doof.set_diameter(0.203); % rocket diameter (m)
Doof.set_radius(Doof.get_diameter()/2);
Doof.set_length(3.4); % length of rocket (m)
Doof.set_initialCG(initial_CG); % initial CG from tip of nosecone (m)
Doof.set_finalCG(final_CG); % final CG from tip of nosecone (m)

wetRollMOI = 0.2109;  % Ixx
wetPitchMOI = 21.1212;  % Iyy
wetYawMOI = 21.1202;  % Izz
dryRollMOI = 0.2036;
dryPitchMOI = 18.99;
dryYawMOI = 18.988;

burn_time = 13.1;

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
aero = Aerodynamics();

syms u v w rho CG p r
FAx_sym = (0.231*((v/u)^2)*rho*(u^2 + v^2 + w^2))/(3.59306 + sqrt(1 - (8.3939e-6*(u^2 + v^2 + w^2)))) + (0.02563*rho*(u^2 + v^2 + w^2)*(v/u)*(asin((r/(u^2+v^2+w^2))*(3.147-CG))))/sqrt(1.7976 - (6.889*10e-6)*(u^2+v^2+w^2)) - 0.08131*rho*((v/u)^2)*(u^2+v^2+w^2) - 0.001364*rho*(u^2 + v^2 + w^2);
FAy_sym = (-0.231*((v/u))*rho*(u^2 + v^2 + w^2))/(3.59306 + sqrt(1 - (8.3939e-6*(u^2 + v^2 + w^2)))) + (-0.02563*rho*(u^2 + v^2 + w^2)*(asin((r/(u^2+v^2+w^2))*(3.147-CG))))/sqrt(1.7976 - (6.889*10e-6)*(u^2+v^2+w^2)) - 0.08131*rho*((v/u)^3)*(u^2+v^2+w^2) - 0.001364*rho*(u^2 + v^2 + w^2)*(v/u);
FAz_sym = 0.01*sin(u); % essentially a noise term
n_a_sym = (3.146*rho*(u^2 + v^2 + w^2))/(27.816 + 16.834*sqrt(1-(8.939*10^-6)*(u^2+v^2+w^2))) - (1.0679*10^-4)*rho*(u^2+v^2+w^2)*(v/u) + (1.695*10^-4)*rho*(u^2+v^2+w^2);
n_r_sym = ((0.0349*rho*(u^2+v^2+w^2))/(1+sqrt(1.798-(6.889*10^-6)*(u^2+v^2+w^2))))*asin((r/(u^2+v^2+w^2))*(3.1473-CG))*(10.3264 - 3.281*CG);
L_p_sym = (0.02064*p)/sqrt(1-(8.6372*10^-6)*(u^2+v^2+w^2));

FAx_func = matlabFunction(FAx_sym);
FAy_func = matlabFunction(FAy_sym);
FAz_func = matlabFunction(FAz_sym);
n_a_func = matlabFunction(n_a_sym);
n_r_func = matlabFunction(n_r_sym);
L_p_func = matlabFunction(L_p_sym);

aero.set_FAx_func(FAx_func);
aero.set_FAy_func(FAy_func);
aero.set_FAz_func(FAz_func);
aero.set_n_a_func(n_a_func);
aero.set_n_r_func(n_r_func);
aero.set_L_p_func(L_p_func);

%% Environment Parameters

env = Environment();
%env.set_launchSiteAltitude(177); % launch site altitude (m) [not implemented]
% TODO - other parameters


%% Flight Parameters

flight = Flight();
flight.set_tStep(0.01); % s
flight.set_runClosedLoop(false);
flight.set_calcXDots(true);
flight.set_runLinearModel(false);
flight.set_applyWind(false);
flight.set_numStates(13);
flight.set_numInputs(3);
flight.set_simTime(12); % seconds
flight.set_localSimTime(12); % linear model propagation time, jet vanes linear model only goes up to 12 seconds
flight.set_tSpan()
%flight.set_tSpan(0:flight.get_tStep():flight.get_simTime());

flight.set_uSpan(zeros(length(flight.get_tSpan()), flight.get_numInputs()));

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
                 0.001; % p
                 0; % q
                 0]; % r
flight.set_initState(initial_state);

% At this point, should be able to run a single nonlinear flight
%flight.run_Flight()
%flight.plot_sim_dashboard()

%% Trim Point Parameters

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

FAx = FAx_sym;
FAy = FAy_sym;
FAz = FAz_sym;
MAx = L_p_sym;
MAy = 0;
MAz = n_a_sym + n_r_sym;

EOM_only_aero = [(1/m)*FAx,
                 (1/m)*FAy,
                 (1/m)*FAz,
                 (1/Ixx)*MAx,
                 (1/Iyy)*MAy,
                 (1/Izz)*MAz,
                 0,
                 0,
                 0]; % This is the only thing that needs to be configured for a new vehicle

CG_t = final_CG + (initial_CG - final_CG) * (burn_time - t) / burn_time;
Ixx_t = dryRollMOI + (wetRollMOI - dryRollMOI) * (burn_time - t) / burn_time;
Iyy_t = dryPitchMOI + (wetPitchMOI - dryPitchMOI) * (burn_time - t) / burn_time;
Izz_t = dryYawMOI + (wetYawMOI - dryYawMOI) * (burn_time - t) / burn_time;
m_t = dry_mass + (wet_mass - dry_mass) * (burn_time - t) / burn_time;

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

A = -1*inv(E)*A_prime % NOTE: The actual formula is -1*inv(E)*A_prime and same for B_prime, however, 
% inv(E) was returning a diagonal matrix of negative infinities. In other
% sections, it was not. This was weird, and because E is a diagonal with
% all -1's, inv(E) = E, so it is just left as -1*E*A_prime
B = -1*inv(E)*B_prime

% Then delta_xdot = A*delta_x + B*delta_u
% xdot_0, x_0, u_0 <--- trim point
% xdot, x, u <--- current rocket state
% delta_xdot = xdot - dxot_0, delta_x = x - x_0, delta_u = u - u0

compute_A_fn = matlabFunction(A);

compute_A = @(u, v, w, p, q, r, q0, q1, q2, q3, t)compute_A_fn(p,q,q0,q1,q2,q3,r,t,u,v,w);
compute_B = matlabFunction(B);


save("master_struct", "compute_B", "compute_A", "Doof", "flight", "aero", "env")

%A_current_time_step = compute_A(2.43,1,10,10,9.81,33,0.02,-0.04,0.962,0,0,0.0872,0.07,0.8,150,0.4,0.02)
%B_current_time_step = compute_B(1,10,10,33)
% The above are function handles that can compute A, B given a current
% rocket state



%% Monte Carlo Parameters

