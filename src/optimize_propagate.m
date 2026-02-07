function [x_next, u_star] = optimize_propagate(x, Q, R, A, B, dt, u_max, eom_fun, thrust_fun, t_now)
% Pick u via linearized RK4-in-cost, but advance the state with your
% nonlinear RK4_propagate using eom_fun + thrust_fun.

num_states   = size(x,1);
num_controls = size(B,2);

Q = (Q+Q')/2;   % keep PSD
R = (R+R')/2;

cvx_begin quiet
    variables u(num_controls)
    expressions x_lin(num_states)

    % One-step RK4 prediction using the *linearized* model for the objective
    F1 = A*x + B*u;
    F2 = A*(x + 0.5*dt*F1) + B*u;
    F3 = A*(x + 0.5*dt*F2) + B*u;
    F4 = A*(x + dt*F3)     + B*u;
    x_lin = x + (dt/6) * (F1 + 2*F2 + 2*F3 + F4);

    minimize( quad_form(x_lin, Q) + quad_form(u, R) )
    norm(u,2) <= u_max;
cvx_end

if ~strcmp(cvx_status,'Solved') && ~strcmp(cvx_status,'Inaccurate/Solved')
    error('CVX failed (%s).', cvx_status);
end
if any(~isfinite(u))
    error('CVX produced non-finite u.');
end

% Advance the *true* state with your nonlinear RK4 (uses thrust_fun inside)
u_star = u;
x_next = RK4_propagate(x, u_star, t_now, dt, eom_fun, thrust_fun, aero);

if any(~isfinite(x_next))
    error('Nonlinear RK4 produced non-finite state.');
end
end
