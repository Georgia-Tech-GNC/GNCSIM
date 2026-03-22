function ocp = build_mpc_solver(num_states, num_controls, Q, R, R_delta, wq, u_max, N, horizon)
import casadi.*

nxA = num_states + num_controls;     
dt_mpc = horizon / N;

% -------------------------
% Symbolics
% -------------------------
xA = SX.sym('xA', nxA, 1);
u  = SX.sym('u',  num_controls, 1);

x      = xA(1:num_states);
u_prev = xA(num_states+1:end);

% parameters per stage: [t0; T; aero; x_ref(13)]
np = 1 + 1 + 1 + num_states;
p  = SX.sym('p', np, 1);

t0    = p(1);
Tpar  = p(2);
aero  = p(3);
x_ref = p(4:4+num_states-1);

q_curr = x(10:13);
q_ref  = x_ref(10:13);

% -------------------------
% DISCRETE dynamics: RK4 for x, and u_prev_next = u
% -------------------------
f = @(x,u,t0,Tpar,aero) eom_nonlin_acados(x, u, t0, Tpar, aero);

k1 = f(x,                  u, t0, Tpar, aero);
k2 = f(x + 0.5*dt_mpc*k1,  u, t0, Tpar, aero);
k3 = f(x + 0.5*dt_mpc*k2,  u, t0, Tpar, aero);
k4 = f(x + 1.0*dt_mpc*k3,  u, t0, Tpar, aero);

x_next  = x + (dt_mpc/6)*(k1 + 2*k2 + 2*k3 + k4);

xA_next = [x_next; u];

% -------------------------
% Model for acados
% -------------------------
model = AcadosModel();
model.name = 'mpc_aug';

model.x = xA;
model.u = u;
model.p = p;

model.disc_dyn_expr = xA_next;

% -------------------------
% Cost: NONLINEAR_LS
% -------------------------
e  = x - x_ref;
du = u - u_prev;

% weighted residuals
sqrtQ = diag(sqrt(diag(Q)));
sqrtR = diag(sqrt(diag(R)));
sqrtRdelta = diag(sqrt(diag(R_delta)));

quat_norm_res = sqrt(wq) * (1 - q_curr.' * q_ref);

q_curr_conj = [q_curr(1);
              -q_curr(2);
              -q_curr(3);
              -q_curr(4)];

q_err = [ q_ref(1)*q_curr_conj(1) - q_ref(2)*q_curr_conj(2) - q_ref(3)*q_curr_conj(3) - q_ref(4)*q_curr_conj(4);
          q_ref(1)*q_curr_conj(2) + q_ref(2)*q_curr_conj(1) + q_ref(3)*q_curr_conj(4) - q_ref(4)*q_curr_conj(3);
          q_ref(1)*q_curr_conj(3) - q_ref(2)*q_curr_conj(4) + q_ref(3)*q_curr_conj(1) + q_ref(4)*q_curr_conj(2);
          q_ref(1)*q_curr_conj(4) + q_ref(2)*q_curr_conj(3) - q_ref(3)*q_curr_conj(2) + q_ref(4)*q_curr_conj(1) ];

quat_vec_res = sqrt(wq)*q_err(2:4);

y = [sqrtQ * e;
     sqrtR * u;
     sqrtRdelta * du;
     quat_norm_res;
     quat_vec_res];

Qe = Q;
Qe(7,7) = 100*Q(7,7);
Qe(8,8) = 100*Q(8,8);
Qe(9,9) = 100*Q(9,9);

wq_e = 10*wq;
sqrtQe = diag(sqrt(diag(Qe)));
quat_norm_res_e = sqrt(wq_e) * (1 - q_curr.' * q_ref);
quat_vec_res_e = sqrt(wq_e) * q_err(2:4);



y_e = [sqrtQe * e;
       quat_norm_res_e;
       quat_vec_res_e];

ny   = length(y);
ny_e = length(y_e);

model.cost_y_expr   = y;
model.cost_y_expr_e = y_e;

ocp = AcadosOcp();
ocp.model = model;

ocp.dims.N = N;

ocp.cost.cost_type   = 'NONLINEAR_LS';
ocp.cost.cost_type_e = 'NONLINEAR_LS';

ocp.cost.W   = eye(ny);
ocp.cost.W_e = eye(ny_e);

ocp.cost.yref   = zeros(ny,1);
ocp.cost.yref_e = zeros(ny_e,1);

% -------------------------
% Constraints
% -------------------------
ocp.constraints.constr_type = 'BGH';

ocp.constraints.lbu   = -u_max * ones(num_controls,1);
ocp.constraints.ubu   =  u_max * ones(num_controls,1);
ocp.constraints.idxbu = (0:num_controls-1)';

% Initial-state constraint placeholders
ocp.constraints.idxbx_0 = (0:nxA-1)';
ocp.constraints.lbx_0   = zeros(nxA,1);
ocp.constraints.ubx_0   = zeros(nxA,1);

% default parameter values
ocp.parameter_values = zeros(np,1);

% -------------------------
% Solver options
% -------------------------
ocp.solver_options.tf = horizon;
ocp.solver_options.integrator_type = 'DISCRETE';

ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM';
ocp.solver_options.nlp_solver_type = 'SQP_RTI';
ocp.solver_options.hessian_approx = 'GAUSS_NEWTON';
ocp.solver_options.nlp_solver_max_iter = 30;

% helpful regularization
ocp.solver_options.levenberg_marquardt = 1e-4;

end