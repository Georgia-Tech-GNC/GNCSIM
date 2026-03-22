% Model Predictive Control
clc
clear solver
clear classes
clear mex
fclose('all')
pause(1)

import casadi.*



num_states = 13; 
num_controls = 3;

run_time = 13.1;
burn_time = 13.1;
dt = .01;

horizon = 1.0; % 1.0, 50 for best, .5, 5 for speed
N = 50;     
dt_mpc = horizon / N;

noise = false;

steps = round(run_time / dt);

aero_bin = 0; % aero on/off
aero_tf = false;
if aero_bin == 1
    aero_tf = true;
end

MPC_data = struct("x",[],"y",[],"z",[],"u",[],"v",[],"w",[],"p",[],"q",[],"r",[], ...
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

u_max = 40; % actually 80, but usually works for 20+

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

ocp = build_mpc_solver(num_states, num_controls, Q, R, R_delta, wq, u_max, N, horizon);

% build solver (generates code + compiles)
solver = AcadosOcpSolver(ocp);

% augmented state init
u_prev = zeros(3,1);
xA_now = [x; u_prev];

X_ws = repmat([x; u_prev], 1, N+1);
U_ws = zeros(num_controls, N);
have_ws = false;

% simulate
tic
for i = 1:steps

    if mod(i,10)==1
    fprintf('step %d / %d\n', i, steps);
    end



    t = dt*(i-1);
    currentThrust = getThrust(t, arr, burn_time);

    Xref_k = repmat(x, 1, N+1);   % start from “hold current state” baseline
    
    for k = 1:(N+1)
        t_pred = t + (k-1)*dt_mpc;
    
        % --- choose quaternion goal based on predicted time ---

        if t_pred < burn_time/3

            % 5 deg tilt
            quat_goal = tilt_from_q(x_init(10:13),5);
            % 5 degree quat_goal is [0.6756;  0;  0.7373; 0];

            % 5 deg spin
            % quat_goal = [0.70643377;  0.03084356;  0.70643377; -0.03084356]; 

        elseif t_pred < 2*burn_time/3
            quat_goal = x_init(10:13);        % back to straight
        else
            % -5 deg tilt
            quat_goal = tilt_from_q(x_init(10:13),-5);

            % - 5 deg spin
            % quat_goal = [0.70643377;  0.03084356;  0.70643377; -0.03084356];
        end

    if dot(x(10:13), quat_goal) < 0
        quat_goal = -quat_goal;
    end

        Xref_k(7:9,k) = [0;0;0];
        Xref_k(10:13,k) = quat_goal;
        
        %disp(quat_goal)
    end

    % 1) Build Xref_k for horizon (your existing logic)
    % Xref_k is 13 x (N+1)
    
    % 2) Update thrust etc. for predicted steps
    % We’ll package per-stage parameters p_k = [t0; T; aero; x_ref]
   
    for k = 0:N-1
        t_pred = t + k*dt_mpc;
        T_pred = getThrust(t_pred, arr, burn_time);
    
        p_k = [t_pred; T_pred; double(aero_bin); Xref_k(:,k+1)];
        solver.set('p', p_k, k);
    
        yref_k = zeros(num_states+2*num_controls+1+3,1);
        % residual: state error, controls, control difference, quat error norm, quat error vector 

        solver.set('cost_y_ref', yref_k, k);
    end
  
    t_pred = t + N*dt_mpc;
    T_pred = getThrust(t_pred, arr, burn_time);
    
    p_N = [t_pred; T_pred; double(aero_bin); Xref_k(:,N+1)];
    solver.set('p', p_N, N);

    yref_e = zeros(num_states+1+3,1);
    % terminal residual: state error, quat error norm, quat error vector

    solver.set('cost_y_ref_e', yref_e, N);
    
    % 3) Set initial condition xA(0) = [x_meas; u_prev]
    xA0 = [x_meas; u_prev];
    
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

   for k = 0:N-1
       X_ws(:,k+1) = solver.get('x', k);
       U_ws(:,k+1) = solver.get('u', k);
   end
   X_ws(:,N+1) = solver.get('x', N);
   have_ws = true;
    
   % 6) Extract first control and predicted next u_prev
   u0 = solver.get('u', 0);
    
   % update u_prev for next MPC step
   u_prev = u0;
    
   % propagate your real sim

   x = RK4_propagate(x, u0, t, dt, eom_fun, thrust_fun, aero_tf);

   x(10:13) = x(10:13) / norm(x(10:13));

   % append to MPC_data

   % MPC_data.t = [MPC_data.t;t];
   MPC_data.x = [MPC_data.x;x(1)]; 
   MPC_data.y = [MPC_data.y;x(2)];
   MPC_data.z = [MPC_data.z;x(3)]; % NED convention: z = -altitude
   MPC_data.u = [MPC_data.u;x(4)];
   MPC_data.v = [MPC_data.v;x(5)];
   MPC_data.w = [MPC_data.w;x(6)];
   MPC_data.p = [MPC_data.p;x(7)];
   MPC_data.q = [MPC_data.q;x(8)];
   MPC_data.r = [MPC_data.r;x(9)];
   MPC_data.q0 = [MPC_data.q0;x(10)];
   MPC_data.q1 = [MPC_data.q1;x(11)];
   MPC_data.q2 = [MPC_data.q2;x(12)];
   MPC_data.q3 = [MPC_data.q3;x(13)];
    
   if i < steps
       MPC_data.roll = [MPC_data.roll;u0(1)];
       MPC_data.pitch = [MPC_data.pitch;u0(2)];
       MPC_data.yaw = [MPC_data.yaw;u0(3)];
   end
end
toc

for i = 0:steps-1
    t_i  = i*dt;
    MPC_data.t = [MPC_data.t;t_i];
end

disp("SIMULATION COMPLETE")
%%
display_results(MPC_data);