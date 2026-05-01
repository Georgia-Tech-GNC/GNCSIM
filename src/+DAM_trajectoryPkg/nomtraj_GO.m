% Nominal Trajectory Generator using acados, adapted from MPC

DAM_trajectoryPkg.setup_project_env(); 

clc
clear solver
clear classes
clear mex 
fclose('all');
pause(1)


%-------------------------------------------------------------------------%
%---------------------------INITIALIZE------------------------------------%
%-------------------------------------------------------------------------%

% log data
nom_traj = struct("x",[],"y",[],"z",[],"u",[],"v",[],"w",[],"p",[],"q",[],"r",[], ...
"q0",[],"q1",[],"q2",[],"q3",[], "t",[], "roll",[], "pitch",[], "yaw",[]);

num_states = 13;  % state dimension
num_controls = 3; % control dimension

run_time = 20.0;  % simulation time
burn_time = 13.1; % how long engine burns
dt = .01;

horizon = run_time;
N = run_time/dt;  % shooting steps: want dt_sim = dt     
dt_sim = horizon / N;

aero_bin = 0;     % aero on/off
aero_tf = false;
if aero_bin == 1
    aero_tf = true;
end

% initial state
x = [0;         % x           
     0;         % y
     0;         % z
     .001;      % u
     0;         % v
     0;         % w
     0;         % p
     0;         % q
     0;         % r
     1/sqrt(2); % q0
     0;         % q1
     1/sqrt(2); % q2
     0];        % q3

x_init = x(:);

u_prev = zeros(num_controls,1);  % initial prev control = 0
xA0 = [x; u_prev]; % initial augmented state

Q = ones(num_states, 1);   % penalizes deviation from x_ref
R = ones(num_controls, 1); % penalizes control magnitude

Q(1) = .1; % x 
Q(2) = .1; % y
Q(3) = .1; % z
Q(4) = .1; % u
Q(5) = .1; % v
Q(6) = .1; % w
Q(7) = 1; % p
Q(8) = 1; % q
Q(9) = 1; % r
Q(10) = 100; % q0
Q(11) = 100; % q1
Q(12) = 100; % q2
Q(13) = 100; % q3

R(1) = 1; % M_roll
R(2) = 1; % M_pitch
R(3) = 1; % M_yaw

Q = diag(1*Q);
R = diag(1*R);

u_max = 10; % actually 80, but can be reduced to prevent large controls

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

%-------------------------------------------------------------------------%
%---------------------------BUILD SOLVER----------------------------------%
%-------------------------------------------------------------------------%

wu = [1e8;1e8;1e8]; % penalty on du
R_delta = diag(wu); 
wq = 1e12;          % penalty on scalar and vector attitude error

ocp = DAM_trajectoryPkg.build_nomtraj_solver(num_states, num_controls, Q, R, R_delta, wq, u_max, N, horizon);
% build solver (generates code + compiles)
solver = AcadosOcpSolver(ocp);

tic

%-------------------------------------------------------------------------%
%-----------------------SET PARAMETERS FOR EACH STEP----------------------%
%-------------------------------------------------------------------------%

% recall parameters are [t0; T; aero; x_ref]

Xref_k = repmat(x, 1, N+1);   % reference states, start from “hold current state” baseline

for k = 1:(N+1)
    t_pred = (k-1)*dt_sim;

    % choose quat goal based on predicted time
   
    if t_pred < burn_time/3
        % 5 degree tilt quat_goal is [0.6756;  0;  0.7373; 0];
        quat_goal = DAM_trajectoryPkg.tilt_from_q(x_init(10:13),5);

        % 5 deg spin
        % quat_goal = [0.70643377;  0.03084356;  0.70643377; -0.03084356];

    elseif t_pred < 2*burn_time/3
        quat_goal = x_init(10:13);        % back to straight

    elseif t_pred > 2*burn_time/3 && t_pred < burn_time
        % -5 deg tilt
        quat_goal = DAM_trajectoryPkg.tilt_from_q(x_init(10:13),-5);

        % - 5 deg spin
        % quat_goal = [0.70643377;  -0.03084356;  0.70643377; 0.03084356];
    else
        quat_goal = x_init(10:13);        % back to straight
    end

 % unnecessary check
 % if dot(x(10:13), quat_goal) < 0
 %     quat_goal = -quat_goal;
 % end

 Xref_k(7:9,k) = [0;0;0]; % goal is omega = 0
 Xref_k(10:13,k) = quat_goal; % attitude goal at step k
end

for k = 0:N-1
    t_pred = k*dt_sim;                                       % t0
    T_pred = DAM_trajectoryPkg.getThrust(t_pred, arr, burn_time);              % T
    p_k = [t_pred; T_pred; double(aero_bin); Xref_k(:,k+1)]; % package params
    solver.set('p', p_k, k); % set kth parameter in solver

    yref_k = zeros(num_states+2*num_controls+1+3,1); 

    % residual: state error, controls, control difference, quat error norm, quat error vector 

    solver.set('cost_y_ref', yref_k, k); % set kth residual
end

% terminal parameters + residual
t_pred = N*dt_sim;
T_pred = DAM_trajectoryPkg.getThrust(t_pred, arr, burn_time);
p_N = [t_pred; T_pred; double(aero_bin); Xref_k(:,N+1)];
solver.set('p', p_N, N);

yref_e = zeros(num_states+1+3,1);

% terminal residual: state error, quat error norm, quat error vector 

solver.set('cost_y_ref_e', yref_e, N);

% Initial conditions/constraints 
solver.set('constr_x0', xA0, 0);

% Initial guesses
for k = 0:N-1
   solver.set('init_x', [Xref_k(:,k+1); u_prev], k); 
   solver.set('init_u', zeros(num_controls,1), k);
end
solver.set('init_x', [Xref_k(:,N+1); u_prev], N);

% Solve
solver.solve();
status = solver.get('status'); 
disp(['acados status = ', num2str(status)]);

% status should be 0. 2 means it reached maximum iterations w/o a solution
% the results are good even though status is 2

%-------------------------------------------------------------------------%
%---------------------------LOG DATA--------------------------------------%
%-------------------------------------------------------------------------%

% time
for i = 0:N
    t_i  = i*dt_sim;
    nom_traj.t = [nom_traj.t;t_i];
end

% controls and states

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
DAM_trajectoryPkg.display_results(nom_traj);