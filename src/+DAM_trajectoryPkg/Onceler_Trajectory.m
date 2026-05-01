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
Onceler_traj = struct("x",[],"y",[],"z",[],"u",[],"v",[],"w",[],"p",[],"q",[],"r",[], ...
"q0",[],"q1",[],"q2",[],"q3",[], "t",[], "beta1a",[], "beta1b",[], "beta2a",[], "beta2b", []);

num_states = 13;  % state dimension
num_controls = 4; % control dimension

run_time = 20.0;  % simulation time
burn_time = 11.94; % how long engine burns
dt = .01;

horizon = run_time;
N = run_time/dt;  % shooting steps: want dt_sim = dt     
dt_sim = horizon / N;

aero_bin = 1;     % aero on/off
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

beta_prev = zeros(num_controls,1);  % initial prev control = 0
xA0 = [x; beta_prev]; % initial augmented state

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

R(1) = 1;
R(2) = 1;
R(3) = 1;
R(4) = 1;

Q = diag(1*Q);
R = diag(1*R);

u_max = deg2rad(10); % conservative, each tab can deflect 10 degrees

% thrust curve
temp = load('+DAM_trajectoryPkg/Onceler_thrust_curve.mat');
arr = temp.thrust_curve_fin_tabs;

%-------------------------------------------------------------------------%
%---------------------------BUILD SOLVER----------------------------------%
%-------------------------------------------------------------------------%

weight_u = 1e2;
wu = [weight_u;weight_u;weight_u;weight_u]; % penalty on du
R_delta = diag(wu); 
wq = 1e12;          % penalty on scalar and vector attitude error

ocp = DAM_trajectoryPkg.build_Onceler_solver(num_states, num_controls, Q, R, R_delta, wq, u_max, N, horizon);
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
   solver.set('init_x', [Xref_k(:,k+1); beta_prev], k); 
   solver.set('init_u', zeros(num_controls,1), k);
end
solver.set('init_x', [Xref_k(:,N+1); beta_prev], N);

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
    Onceler_traj.t = [Onceler_traj.t;t_i];
end

% controls and states

for i = 0:N
    if i < N
        u_i = solver.get('u', i);
        Onceler_traj.beta1a = [Onceler_traj.beta1a;u_i(1)];
        Onceler_traj.beta1b = [Onceler_traj.beta1b;u_i(2)];
        Onceler_traj.beta2a = [Onceler_traj.beta2a;u_i(3)];
        Onceler_traj.beta2b = [Onceler_traj.beta2b;u_i(4)];
    end
    x_i = solver.get('x',i);
    Onceler_traj.x = [Onceler_traj.x;x_i(1)]; 
    Onceler_traj.y = [Onceler_traj.y;x_i(2)];
    Onceler_traj.z = [Onceler_traj.z;-x_i(3)]; 
    Onceler_traj.u = [Onceler_traj.u;x_i(4)];
    Onceler_traj.v = [Onceler_traj.v;x_i(5)];
    Onceler_traj.w = [Onceler_traj.w;x_i(6)];
    Onceler_traj.p = [Onceler_traj.p;x_i(7)];
    Onceler_traj.q = [Onceler_traj.q;x_i(8)];
    Onceler_traj.r = [Onceler_traj.r;x_i(9)];
    Onceler_traj.q0 = [Onceler_traj.q0;x_i(10)];
    Onceler_traj.q1 = [Onceler_traj.q1;x_i(11)];
    Onceler_traj.q2 = [Onceler_traj.q2;x_i(12)];
    Onceler_traj.q3 = [Onceler_traj.q3;x_i(13)];
end
toc
disp("SIMULATION COMPLETE")
%%
DAM_trajectoryPkg.display_Onceler(Onceler_traj);