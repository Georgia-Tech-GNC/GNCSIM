function u0 = CasADI_Optimize(num_states,num_controls, x0, x_ref, Q, R, burn_time, dt, u_max, horizon, N, t_now, T_now, aero_now)

import casadi.*


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
% f = Function('f', {x_sym,u_sym,t_sym,T_sym,aero_sym},{xdot});

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

% sim = F.mapaccum(N);

% Optimization problem


opti = casadi.Opti();

X = opti.variable(num_states, N+1); % N+1 states
U = opti.variable(num_controls, N); % N controls

x_initial = opti.parameter(num_states,1);
opti.subject_to(X(:,1) == x_initial);

dt_mpc = horizon/N;

% Dynamics constraints
for k = 1:N
    t_k = t_now + (k-1)*dt_mpc;  % numeric time for stage k
    opti.subject_to( X(:,k+1) == F(X(:,k), U(:,k), t_k, T_now, aero_now) );
end

% Control magnitude constraint
for k = 1:N
    opti.subject_to( U(1,k)^2 + U(2,k)^2 + U(3,k)^2 <= u_max^2 );
end

% Cost: sum stage + terminal 
J = 0;
for k = 1:N % STEP COST (x-x_ref)^T Q (x-x_ref) + u^T R u
    J = J + (X(:,k) - x_ref).' * Q * (X(:,k) - x_ref) + U(:,k).' * R * U(:,k); 
end

% TERMINAL COST
J = J + (X(:,N+1) - x_ref).' * Q * (X(:,N+1) - x_ref);

opti.minimize(J);

opti.set_value(x_initial, x0);

opti.set_initial(U, 0);                     % initial guesses for state + control
opti.set_initial(X, repmat(x0,1,N+1));


opti.solver('ipopt');   % choose solver

sol = opti.solve();     % solve  

u0 = sol.value(U(:,1));   % first control in optimal sequence
end