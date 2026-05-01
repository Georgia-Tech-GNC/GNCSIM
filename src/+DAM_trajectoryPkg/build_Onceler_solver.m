function ocp = build_Onceler_solver(num_states, num_controls, Q, R, R_delta, wq, u_max, N, horizon)

% build the solver for our optimization problem

import casadi.*

nxA = num_states + num_controls;    % dimension of augmented state  
dt_nomtraj = horizon / N;           % timestep

%-------------------------------------------------------------------------%
%---------------------------CASADI SYMBOLICS------------------------------%
%-------------------------------------------------------------------------%

xA = casadi.SX.sym('xA', nxA, 1);          % augmented state: [state; previous control]
beta_vec  = casadi.SX.sym('u',  num_controls, 1); % control (tab deflections)

x      = xA(1:num_states);          % state
beta_prev = xA(num_states+1:end);      % previous control (to penalize jerky controls)

np = 1 + 1 + 1 + num_states;        % dimension of time-varying parameters
params  = casadi.SX.sym('params', np, 1);  % time-varying parameters

t0    = params(1);                  % current time
Tpar  = params(2);                  % thrust
aero  = params(3);                  % aerodynamics on/off
x_ref = params(4:4+num_states-1);   % reference (goal) state

q_curr = x(10:13);                  % current quaternion
q_ref  = x_ref(10:13);              % reference quaternion

%-------------------------------------------------------------------------%
%---------------------------RK4 DYNAMICS----------------------------------%
%-------------------------------------------------------------------------%

f = @(x,beta_vec,t0,Tpar,aero) DAM_trajectoryPkg.Onceler_eom_acados(x, beta_vec, t0, Tpar, aero); % symbolic dynamics

k1 = f(x,                  beta_vec, t0, Tpar, aero); % RK4 coefficients
k2 = f(x + 0.5*dt_nomtraj*k1,  beta_vec, t0, Tpar, aero);
k3 = f(x + 0.5*dt_nomtraj*k2,  beta_vec, t0, Tpar, aero);
k4 = f(x + 1.0*dt_nomtraj*k3,  beta_vec, t0, Tpar, aero);

x_next  = x + (dt_nomtraj/6)*(k1 + 2*k2 + 2*k3 + k4); % propagated state

xA_next = [x_next; beta_vec]; % attach the previous control

% ------------------------------------------------------------------------%
% --------------------------ACADOS MODEL----------------------------------%
% ------------------------------------------------------------------------%

model = AcadosModel();
model.name = 'nomtraj_aug';

model.x = xA;                       % the model's state
model.u = beta_vec;                        % the model's control
model.p = params;                   % the model's parameters

model.disc_dyn_expr = xA_next;      % next state in terms of current state, control, parameters

%-------------------------------------------------------------------------%
%---------------------------NONLINEAR COST FUNCTION-----------------------%
%-------------------------------------------------------------------------%

x_err  = x - x_ref; % deviation from reference state
du = beta_vec - beta_prev;    % control difference

sqrtQ = diag(sqrt(diag(Q)));
sqrtR = diag(sqrt(diag(R)));
sqrtRdelta = diag(sqrt(diag(R_delta)));

quat_norm_res = sqrt(wq) * (1 - q_curr.' * q_ref); % weighted scalar attitude error

q_curr_conj = [q_curr(1);
              -q_curr(2);
              -q_curr(3);
              -q_curr(4)];

q_err = DAM_trajectoryPkg.quatmul(q_ref,q_curr_conj);

quat_vec_res = sqrt(wq)*q_err(2:4); % weighted vector part of attitude error

% build stage residual, dimension = num_states + 2*num_controls + 1 + 3

y = [sqrtQ * x_err;   % (...)'*(...) = x_err*Q*x_err
     sqrtR * beta_vec;       % (...)'*(...) = u*R*u
     sqrtRdelta * du; % (...)'*(...) = du*R_delta*x_du
     quat_norm_res;   % minimize scalar attitude error
     quat_vec_res];   % minimize vector attitude error

% build terminal residual

Qe = Q;
Qe(7,7) = 100*Q(7,7);
Qe(8,8) = 100*Q(8,8);
Qe(9,9) = 100*Q(9,9);

wq_e = 10*wq;
sqrtQe = diag(sqrt(diag(Qe)));
quat_norm_res_e = sqrt(wq_e) * (1 - q_curr.' * q_ref);
quat_vec_res_e = sqrt(wq_e) * q_err(2:4);



y_e = [sqrtQe * x_err;
       quat_norm_res_e;
       quat_vec_res_e];

ny   = length(y);
ny_e = length(y_e);

model.cost_y_expr   = y;   % set stage residual
model.cost_y_expr_e = y_e; % set terminal residual

ocp = AcadosOcp();
ocp.model = model;

ocp.dims.N = N;

ocp.cost.cost_type   = 'NONLINEAR_LS';
ocp.cost.cost_type_e = 'NONLINEAR_LS';

% solver minimizes y^T * W * y, in our construction W = I

ocp.cost.W   = eye(ny);
ocp.cost.W_e = eye(ny_e);

ocp.cost.yref   = zeros(ny,1);
ocp.cost.yref_e = zeros(ny_e,1);

%-------------------------------------------------------------------------%
%---------------------------CONSTRAINTS-----------------------------------%
%-------------------------------------------------------------------------%
ocp.constraints.constr_type = 'BGH'; % bounds constraint

ocp.constraints.lbu   = -u_max * ones(num_controls,1); % box constraints on u
ocp.constraints.ubu   =  u_max * ones(num_controls,1);
ocp.constraints.idxbu = (0:num_controls-1)';           % indices for constrained u

% To fix our initial state 
ocp.constraints.idxbx_0 = (0:nxA-1)';                  % indices for constrained x
ocp.constraints.lbx_0   = zeros(nxA,1);                % placeholder bounds
ocp.constraints.ubx_0   = zeros(nxA,1);

% default parameter values (set them later)
ocp.parameter_values = zeros(np,1);

%-------------------------------------------------------------------------%
%---------------------------SOLVER OPTIONS--------------------------------%
%-------------------------------------------------------------------------%

ocp.solver_options.tf = horizon;
ocp.solver_options.integrator_type = 'DISCRETE';

ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM';
ocp.solver_options.nlp_solver_type = 'SQP'; % SQP_RTI better for MPC
ocp.solver_options.hessian_approx = 'GAUSS_NEWTON';
ocp.solver_options.nlp_solver_max_iter = 30;

ocp.solver_options.levenberg_marquardt = 1e-4; % Keep the problem well-conditioned

end