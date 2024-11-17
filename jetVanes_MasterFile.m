%% Master GNCSIM Simulation File for Jet Vanes Rocket

%% Rocket Parameters

Doof = Rocket();
Doof.set_wetMass(38.87);
Doof.set_dryMass(30.96);
Doof.set_diameter(0.203); % rocket diameter (m)
Doof.set_radius(Doof.get_diameter()/2);
Doof.set_length(3.4); % length of rocket (m)
Doof.set_initialCG(2.433); % initial CG from tip of nosecone (m)
Doof.set_finalCG(2.34); % final CG from tip of nosecone (m)

wetRollMOI = 0.2109;
wetPitchMOI = 21.1212;
wetYawMOI = 21.1202;
dryRollMOI = 0.2036;
dryPitchMOI = 18.99;
dryYawMOI = 18.988;

Doof.set_wetMOI([wetRollMOI 0 0; 0 wetPitchMOI 0; 0 0 wetYawMOI]);
Doof.set_dryMOI([dryRollMOI 0 0; 0 dryPitchMOI 0; 0 0 dryYawMOI]);
Doof.set_burnTime(13.1);

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
FAx = (0.231*((v/u)^2)*rho*(u^2 + v^2 + w^2))/(3.59306 + sqrt(1 - (8.3939e-6*(u^2 + v^2 + w^2)))) + (0.02563*rho*(u^2 + v^2 + w^2)*(v/u)*(asin((r/(u^2+v^2+w^2))*(3.147-CG))))/sqrt(1.7976 - (6.889*10e-6)*(u^2+v^2+w^2)) - 0.08131*rho*((v/u)^2)*(u^2+v^2+w^2) - 0.001364*rho*(u^2 + v^2 + w^2);
FAy = (-0.231*((v/u))*rho*(u^2 + v^2 + w^2))/(3.59306 + sqrt(1 - (8.3939e-6*(u^2 + v^2 + w^2)))) + (-0.02563*rho*(u^2 + v^2 + w^2)*(asin((r/(u^2+v^2+w^2))*(3.147-CG))))/sqrt(1.7976 - (6.889*10e-6)*(u^2+v^2+w^2)) - 0.08131*rho*((v/u)^3)*(u^2+v^2+w^2) - 0.001364*rho*(u^2 + v^2 + w^2)*(v/u);
FAz = 0.01*sin(u); % essentially a noise term
n_a = (3.146*rho*(u^2 + v^2 + w^2))/(27.816 + 16.834*sqrt(1-(8.939*10^-6)*(u^2+v^2+w^2))) - (1.0679*10^-4)*rho*(u^2+v^2+w^2)*(v/u) + (1.695*10^-4)*rho*(u^2+v^2+w^2);
n_r = ((0.0349*rho*(u^2+v^2+w^2))/(1+sqrt(1.798-(6.889*10^-6)*(u^2+v^2+w^2))))*asin((r/(u^2+v^2+w^2))*(3.1473-CG))*(10.3264 - 3.281*CG);
L_p = (0.02064*p)/sqrt(1-(8.6372*10^-6)*(u^2+v^2+w^2));

FAx_func = matlabFunction(FAx);
FAy_func = matlabFunction(FAy);
FAz_func = matlabFunction(FAz);
n_a_func = matlabFunction(n_a);
n_r_func = matlabFunction(n_r);
L_p_func = matlabFunction(L_p);

aero.set_FAx_func(FAx_func);
aero.set_FAy_func(FAy_func);
aero.set_FAz_func(FAz_func);
aero.set_n_a_func(n_a_func);
aero.set_n_r_func(n_r_func);
aero.set_L_p_func(L_p_func);

%% Environment Parameters

env = Environment();
env.set_launchSiteAltitude(177); % launch site altitude (m)
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
flight.set_tSpan(0:flight.get_tStep():flight.get_simTime());

flight.set_uSpan(zeros(length(flight.get_tSpan(), flight.get_numInputs())));

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


%% Monte Carlo Parameters