function [x_next, u_star] = aop2(x, Q, R, A, B, dt, u_max, eom_fun, thrust_fun, t_now, burn_time, aero)
% Pick u via linearized RK4-in-cost, but advance the state with your
% nonlinear RK4_propagate using eom_fun + thrust_fun.

num_states   = size(x,1);
num_controls = size(B,2);

% reference (desired) state

x_ref = 1*x;
% x_ref(7) = 1.5;
% x_ref(8) = 1;
% x_ref(9) = .1;



% location
x_ref(1) = 5;
x_ref(2) = 5;
x_ref(4) = 1;
x_ref(5) = 1;
x_ref(7) = 1.5;
x_ref(8) = .5;
x_ref(9) = .5;

% 20 degree turn
% x_ref(10) = .9848;
% x_ref(11) = 0;
% x_ref(12) = 0;
% x_ref(13) = .1736;


% back and forth

% x_ref(1) = x_ref(1) - 1;
% x_ref(2) = x_ref(2) - 1;
% 
% if t_now >= burn_time/2
%     x_ref(1) = x_ref(1) + 1;
%     x_ref(2) = x_ref(2) + 1;
% end



% S = (Q+Q')/2;   % keep PSD
% R = (R+R')/2;

% Uncomment these if no matrix exponential
A_d = A * dt + eye(num_states);
B_d = B * dt;

R_inv = inv(R);

N = 10;

u = finite_Ricatti(A_d,B_d,Q,R,x,x_ref,N);


% e = x - x_ref;
% c = (A_d - eye(num_states)) * x_ref;
% u = -(R + B_d.'*Q*B_d) \ (B_d.'*Q*(A_d*e + c));
% u = - inv(R + B_d.' * Q * B_d) * B_d.' * Q * (A_d * x-x_ref);



% Advance the *true* state with your nonlinear RK4 (uses thrust_fun inside)
u_star = u;
x_next = RK4_propagate(x, u_star, t_now, dt, eom_fun, thrust_fun, aero);

if any(~isfinite(x_next))
    error('Nonlinear RK4 produced non-finite state.');
end
end
