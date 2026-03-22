% Nominal Trajectory
clc
clear solver
clear classes
clear mex
fclose('all')
pause(1)

import casadi.*

num_states = 13; 
num_controls = 3;

run_time = 20.0;
burn_time = 13.1;
dt = .01;

horizon = run_time; % 1.0, 50 for best, .5, 5 for speed
N = run_time/dt;     
dt_sim = horizon / N;

aero_bin = 0; % aero on/off
aero_tf = false;
if aero_bin == 1
    aero_tf = true;
end

nom_traj = struct("x",[],"y",[],"z",[],"u",[],"v",[],"w",[],"p",[],"q",[],"r",[], ...
"q0",[],"q1",[],"q2",[],"q3",[], "t",[],"T",[], "roll",[], "pitch",[], "yaw",[]);

x = [0; % initial state
     0;
     0;
     .001;
     0;
     0;
     0;
     0;
     0;
     1/sqrt(2);
     0;
     1/sqrt(2);
     0];

x_init = x(:);

Q = ones(num_states, 1);
R = ones(num_controls, 1);

% tune for each x_ref
Q(1) = .1; % x 
Q(2) = .1; % y
Q(3) = .1; % z
Q(4) = .1; % u
Q(5) = .1; % v
Q(6) = .1; % w
Q(7) = 1e4; % p
Q(8) = 1e4; % q
Q(9) = 1e4; % r
Q(10) = 100; % q0
Q(11) = 100; % q1
Q(12) = 100; % q2
Q(13) = 100; % q3

R(1) = 1; % M_roll
R(2) = 1; % M_pitch
R(3) = 1; % M_yaw

Q = diag(1*Q);
R = diag(1*R);

u_max = 80; % actually 80, but reduced to prevent large oscillation

% thrust curve
arr = [0.0, 0.457288;
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

eom_fun     = @eom_nonlin;                          
thrust_fun  = @(tt) getThrust(tt, arr, burn_time); 

% build solver
wu = [1e8;1e8;1e8];
R_delta = diag(wu);
wq = 1e12;

ocp = build_nomtraj_solver(num_states, num_controls, Q, R, R_delta, wq, u_max, N, horizon);
% build solver (generates code + compiles)
solver = AcadosOcpSolver(ocp);

% augmented state init
u_prev = zeros(3,1);
xA_now = [x; u_prev];

X_ws = repmat([x; u_prev], 1, N+1);
U_ws = zeros(num_controls, N);
have_ws = false;

tic

Xref_k = repmat(x, 1, N+1);   % start from “hold current state” baseline

for k = 1:(N+1)
    t_pred = (k-1)*dt_sim;
    % --- choose quaternion goal based on predicted time ---
    %quat_goal = tilt_from_q(x_init(10:13),5);
    % 5 degree tilt quat_goal is [0.6756;  0;  0.7373; 0];
    quat_goal = [0.70643377;  0.03084356;  0.70643377; -0.03084356];
    if t_pred < burn_time/3
        % 5 deg tilt
        % quat_goal = tilt_from_q(x_init(10:13),5);
        % 5 deg spin
        quat_goal = [0.70643377;  0.03084356;  0.70643377; -0.03084356]; 

    elseif t_pred < 2*burn_time/3
        quat_goal = x_init(10:13);        % back to straight

    elseif t_pred > 2*burn_time/3 && t_pred < burn_time
        % -5 deg tilt
        % quat_goal = tilt_from_q(x_init(10:13),-5);
        % - 5 deg spin
        % quat_goal = [0.70643377;  -0.03084356;  0.70643377; 0.03084356];
    else
        quat_goal = x_init(10:13);        % back to straight
    end

 if dot(x(10:13), quat_goal) < 0
     quat_goal = -quat_goal;
 end

 % alpha = .2;
 % error_angle = 0;
 % [omega, quat_f] = omega_for_quat(Xref_k(:,k), quat_goal, alpha, error_angle, dt);

 % Xref_k(7:9,k) = omega;
 Xref_k(7:9,k) = [0;0;0];
 Xref_k(10:13,k) = quat_goal;
end
% 2) Update thrust etc. for predicted steps
% We’ll package per-stage parameters p_k = [t0; T; aero; x_ref]
for k = 0:N-1
    t_pred = k*dt_sim;
    T_pred = getThrust(t_pred, arr, burn_time);
    p_k = [t_pred; T_pred; double(aero_bin); Xref_k(:,k+1)];
    solver.set('p', p_k, k);

    yref_k = zeros(num_states+2*num_controls+1+3,1);
    % residual: state error, controls, control difference, quat error norm, quat error vector 

    solver.set('cost_y_ref', yref_k, k);
end

t_pred = N*dt_sim;
T_pred = getThrust(t_pred, arr, burn_time);
p_N = [t_pred; T_pred; double(aero_bin); Xref_k(:,N+1)];
solver.set('p', p_N, N);

yref_e = zeros(num_states+1+3,1);
% terminal residual: state error, quat error norm, quat error vector

solver.set('cost_y_ref_e', yref_e, N);

% 3) Set initial condition xA(0) = [x_meas; u_prev]
xA0 = [x; u_prev];
solver.set('constr_x0', xA0, 0);

if have_ws
   for k = 0:N-1
       solver.set('init_x', X_ws(:,k+1), k);
       solver.set('init_u', U_ws(:,k+1), k);
   end
   solver.set('init_x', X_ws(:,N+1), N);
else
   for k = 0:N-1
       solver.set('init_x', [Xref_k(:,k+1); u_prev], k);
       solver.set('init_u', zeros(num_controls,1), k);
   end
   solver.set('init_x', [Xref_k(:,N+1); u_prev], N);
end

% 4) Solve
solver.solve();
status = solver.get('status');
disp(['acados status = ', num2str(status)]);

for i = 0:N
    t_i  = i*dt_sim;
    nom_traj.t = [nom_traj.t;t_i];
end

for i = 0:N
    if i < N
        u_i = solver.get('u', i);
        nom_traj.roll = [nom_traj.roll;u_i(1)];
        nom_traj.pitch = [nom_traj.pitch;u_i(2)];
        nom_traj.yaw = [nom_traj.yaw;u_i(3)];
    end
    x_i = solver.get('x',i);
    nom_traj.x = [nom_traj.x;x_i(1)]; 
    nom_traj.y = [nom_traj.y;x_i(2)];
    nom_traj.z = [nom_traj.z;x_i(3)]; 
    nom_traj.u = [nom_traj.u;x_i(4)];
    nom_traj.v = [nom_traj.v;x_i(5)];
    nom_traj.w = [nom_traj.w;x_i(6)];
    nom_traj.p = [nom_traj.p;x_i(7)];
    nom_traj.q = [nom_traj.q;x_i(8)];
    nom_traj.r = [nom_traj.r;x_i(9)];
    nom_traj.q0 = [nom_traj.q0;x_i(10)];
    nom_traj.q1 = [nom_traj.q1;x_i(11)];
    nom_traj.q2 = [nom_traj.q2;x_i(12)];
    nom_traj.q3 = [nom_traj.q3;x_i(13)];
end
toc
disp("SIMULATION COMPLETE")
%%
display_results(nom_traj);