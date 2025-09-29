function [x_opt, u_in] = optimize_propagate(x,Q,R,A,B,dt,u_max)

 num_states   = size(x,1);    % |x|
 num_controls = size(B,2);    % |u|

% Q is num_states x num_states
% R is num_controls x num_controls
% A is num_states x num_states
% B is num_states x num_controls
% Dynamics: x_dot = A x + B u

% RK4 method
% F1 = A*x + B*u
% F2 = A*(x + 0.5*dt*F1) + B*u
% F3 = A*(x + 0.5*dt*F2) + B*u
% F4 = A*(x + dt*F3)     + B*u
% x_new = x + (dt/6) * (F1 + 2*F2 + 2*F3 + F4)
% x_new = x + ((dt^2/2)*A + (dt^3/6)*(A*A) + (dt^4/24)*(A*A*A))(A*x + B*u)

% ---- CVX solve ----
Q = (Q+Q')/2;    % ensure symmetric (PSD) for DCP
R = (R+R')/2;

Ax  = A*x;          % purely numeric (13x1)
AB  = A*B;          % numeric (13x4)

cvx_begin quiet
    % cvx_solver mosek   % (optional)
    variables u(num_controls)
    expressions x_new(num_states)

    % RK4
    F1 = A*x + B*u;
    F2 = A*(x + 0.5*dt*F1) + B*u;
    F3 = A*(x + 0.5*dt*F2) + B*u;
    F4 = A*(x + dt*F3)     + B*u;
    x_new = x + (dt/6) * (F1 + 2*F2 + 2*F3 + F4);

    % Cost
    minimize( quad_form(x_new, Q) + quad_form(u, R) )

    % Control constraint: ||u|| <= u_max   (equiv. u'*u <= u_max^2)
    norm(u,2) <= u_max;
cvx_end

if ~strcmp(cvx_status,'Solved') && ~strcmp(cvx_status,'Inaccurate/Solved')
    error('CVX failed (%s). Try relaxing constraints / scaling Q,R or u_max.', cvx_status);
end

if any(~isfinite(u)) || any(~isfinite(x_new))
    error('CVX produced non-finite u or x_new.');
end

u_in  = u;
x_opt = x_new;
end