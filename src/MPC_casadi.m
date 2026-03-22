% Model Predictive Control

import casadi.*



num_states = 13; 
num_controls = 3;

run_time = 13.1;
burn_time = 13.1;
dt = .01;

horizon = .5; % 1.0, 50 for best, .5, 5 for speed
N = 5;        

noise = true;

steps = round(run_time / dt);

aero_bin = 0; % aero on/off
aero_tf = false;
if aero_bin == 1
    aero_tf = true;
end

MPC_data = struct("x",[0],"y",[0],"z",[0],"u",[0],"v",[0.01],"w",[0],"p",[0],"q",[0],"r",[0], ...
"q0",[sqrt(2)/2],"q1",[0],"q2",[sqrt(2)/2],"q3",[0], "t",[0],"T",[0], "roll",[0], "pitch",[0], "yaw",[0]);

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
Q(1) = .01; % x 
Q(2) = .01; % y
Q(3) = .01; % z
Q(4) = .01; % u
Q(5) = .01; % v
Q(6) = .01; % w
Q(7) = 1000; % p
Q(8) = 1000; % q
Q(9) = 1000; % r
Q(10) = 1000; % q0
Q(11) = 1000; % q1
Q(12) = 1000; % q2
Q(13) = 1000; % q3

R(1) = 10; % M_roll
R(2) = 10; % M_pitch
R(3) = 10; % M_yaw


Q = diag(1*Q);
R = diag(1*R);


u_max = 80; % maximum magnitude of u

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




% PARAMETERS
t_sym    = MX.sym('t_sym', 1, 1);     % time
T_sym    = MX.sym('T_sym', 1, 1);     % thrust magnitude
aero_sym = MX.sym('aero_sym', 1, 1);  % 0 or 1 flag

% STATE (13×1)
x_sym = MX.sym('x', num_states, 1);

% CONTROL (3×1 moments)
u_sym = MX.sym('u', num_controls, 1);

% build EOM function
xdot = eom_nonlin_casadi(x_sym,u_sym,t_sym,T_sym,aero_sym);

% build integrator


intg_options = struct;
intg_options.tf = horizon/N;
intg_options.simplify = true;
intg_options.number_of_finite_elements = 1;

dae = struct;
dae.x = x_sym; % CasADI state
dae.p = [u_sym; t_sym; T_sym; aero_sym]; % CasADI parameters (fixed during integration)
dae.ode = xdot;

intg = integrator('intg', 'rk', dae, intg_options);

Fk = intg('x0', x_sym, 'p', [u_sym; t_sym; T_sym; aero_sym]);
x_next_sym = Fk.xf;

F = Function('F', {x_sym, u_sym, t_sym, T_sym, aero_sym}, {x_next_sym});

opti = casadi.Opti();

X = opti.variable(num_states, N+1); % N+1 states
U = opti.variable(num_controls, N); % N controls

% Parameters (constant during integration)

x_initial = opti.parameter(num_states,1);
t0      = opti.parameter(1,1);   % current time
Tpar    = opti.parameter(1,1);   % current thrust
aeropar = opti.parameter(1,1);   % 0/1 aero flag
%x_ref_par = opti.parameter(num_states,1);
x_ref_par = opti.parameter(num_states,N+1);
u_prev = opti.parameter(num_controls,1);

opti.subject_to(X(:,1) == x_initial);

dt_mpc = horizon/N;


for k = 1:N
    opti.subject_to( X(:,k+1) == F(X(:,k), U(:,k), t0, Tpar, aeropar) );
end


% Control magnitude constraint
for k = 1:N
    opti.subject_to( U(1,k)^2 + U(2,k)^2 + U(3,k)^2 <= u_max^2 );
end

% Cost: sum stage + terminal 
J = 0;
for k = 1:N % STEP COST (x-x_ref)^T Q (x-x_ref) + u^T R u
    J = J + (X(:,k) - x_ref_par(:,k)).' * Q * (X(:,k) - x_ref_par(:,k)) + U(:,k).' * R * U(:,k); 

    % quaternion vectors
    q_curr = X(10:13, k);
    q_ref = x_ref_par(10:13,k);
    
    % geometric quaternion error
    att_err = 1 - (q_curr.' * q_ref)^2;

    % error weight
    wq = 1e7;  
    J = J + wq * att_err;

    % penalize jerky controls
    
    wu = [1e4;1e4;1e4];
    R_delta = diag(wu);

    if k == 1 % first step
        J = J + (U(:,1) - u_prev).' * R_delta * (U(:,1) - u_prev);
    elseif k >= 2
        J = J + (U(:,k) - U(:,k-1)).' * R_delta * (U(:,k) - U(:,k-1));
    end

   
end


% TERMINAL COST
eN = X(:,N+1) - x_ref_par(:,N+1);     
J  = J + eN.' * Q * eN;

qN = X(10:13, N+1);

att_err_N = 1 - (qN.' * q_ref)^2;
J = J + wq * att_err_N;


opti.minimize(J);


opti.solver('ipopt');   % choose solver (sqpmethod or ipopt)


u_last = zeros(num_controls,1);

% for warm start
U_ws = zeros(num_controls, N);        
X_ws = repmat(x, 1, N+1);             
have_ws = false;                      




% simulate
tic
for i = 1:steps

    if mod(i,10)==1
    fprintf('step %d / %d\n', i, steps);
    end



    t = dt*(i-1);
    currentThrust = getThrust(t, arr, burn_time);
  
   

    

    x_meas = 1 * x; % perturbed state controller sees

    sigma_omega = 0.05;                 % rad/s, angulars have max magnitude ~ .06
    % tilt only gets disrupted for .1 and above
    % spin controls are jerky on a small scale for .01, .005, quats all good
    % .05 max


    
    if noise
        x_meas(7:9) = x_meas(7:9) + sigma_omega*randn(3,1);
    end


    Xref_k = repmat(x, 1, N+1);   % start from “hold current state” baseline
    
    for k = 1:(N+1)
        t_pred = t + (k-1)*dt_mpc;
    
        % --- choose quaternion goal based on predicted time ---

        quat_goal = tilt_from_q(x_init(10:13),5);
        % quat_goal = [0.70643377;  0.03084356;  0.70643377; -0.03084356];

        if t_pred < burn_time/3

            % 5 deg tilt
            quat_goal = tilt_from_q(x_init(10:13),5);

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

        alpha = .2;
        error_angle = 0;
        [omega, quat_f] = omega_for_quat(Xref_k(:,k), quat_goal, alpha, error_angle, dt);

        Xref_k(7:9,k) = omega;
        Xref_k(10:13,k) = quat_f;
    end

    opti.set_value(x_initial, x_meas);        % current measured/estimated state
    opti.set_value(t0, t);           % current time 
    opti.set_value(Tpar, currentThrust); % thrust
    opti.set_value(aeropar, double(aero_bin)); 
    opti.set_value(x_ref_par, Xref_k);  
    opti.set_value(u_prev,u_last);

    if have_ws
        % use previous solution for warm start
        U_init = [U_ws(:,2:end), U_ws(:,end)];    % shift controls left
        X_init = [x, X_ws(:,2:end)];              % shift states left, keep length N+1
        opti.set_initial(U, U_init);
        opti.set_initial(X, X_init);

    else
        % First iteration fallback
        opti.set_initial(U, zeros(num_controls, N));
        opti.set_initial(X, repmat(x, 1, N+1));
    end



    sol = opti.solve();     % solve  

    u0 = sol.value(U(:,1));   % first control in optimal sequence
    u_last = u0(:);

    U_ws = sol.value(U);
    X_ws = sol.value(X);
    have_ws = true;



    
    x = RK4_propagate(x, u0, t, dt, eom_fun, thrust_fun, aero_tf);


    x(10:13) = x(10:13) / norm(x(10:13));



    % append to MPC_data

    MPC_data.t = [MPC_data.t;t];
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
    
    MPC_data.roll = [MPC_data.roll;u0(1)];
    MPC_data.pitch = [MPC_data.pitch;u0(2)];
    MPC_data.yaw = [MPC_data.yaw;u0(3)];
end
toc

disp("SIMULATION COMPLETE")
%%
display_results(MPC_data);