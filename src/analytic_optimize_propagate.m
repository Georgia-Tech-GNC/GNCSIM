function [x_next, u_star] = analytic_optimize_propagate(x, Q, R, A, B, dt, u_max, eom_fun, thrust_fun, t_now, burn_time, aero)
% Pick u via linearized RK4-in-cost, but advance the state with your
% nonlinear RK4_propagate using eom_fun + thrust_fun.

num_states   = size(x,1);
num_controls = size(B,2);

% reference (desired) state, must change p,q,r for nonzero controls

x_ref = 1*x;

% quat_goal = [0.70643377;  0.03084356;  0.70643377; -0.03084356]; % 5 degree spin

% quat_goal = [0.73727734; 0; 0.67559021; 0]; % 5 tilt
 

% quat_goal = [0.38268343; 0; 0.92387953; 0]; % 45 degree tilt

quat_goal = [0.67559021; 0; 0.73727734; 0]; % 5 degree pitch

% if t_now >= burn_time/3 && t_now < 2*burn_time/3
%     quat_goal = [1/sqrt(2); 0; 1/sqrt(2); 0];
% else
%     quat_goal = [0.65328148; -0.27059805; 0.65328148; 0.27059805];
%     % quat_goal = [0.38268343; 0; -0.92387953; 0]; % -45 degree pitch
% end


alpha = 0.1; % bigger means more aggressive control
             % 0.1 worked for 45 deg yaw

error_angle = 0; % set alpha = 1 when attitude error (deg) < error_angle. 0 will turn off this feature

[omega, quat_f] = omega_for_quat(x, quat_goal, alpha, error_angle, dt);

%[omega, ~] = ofq2(x, quat_goal);

x_ref(7:9) = omega; % set angulars in ref state
 
x_ref(10:13) = quat_f; 


% Comment when using matrix exponential
A = A * dt + eye(num_states);
B = B * dt;

R_inv = inv(R);

%tic
thing = inv((eye(num_states)+Q*B*R_inv*B.'));
%toc
u = -R_inv*B.'* thing * Q * (A * x - x_ref); % analytic solution

if norm(u) > u_max
    disp(norm(u))
    u = u / norm(u) * u_max;
end



% Advance the *true* state with your nonlinear RK4 (uses thrust_fun inside)
u_star = u;

x_next = RK4_propagate(x, u_star, t_now, dt, eom_fun, thrust_fun, aero);

x_next(10:13) = x_next(10:13) / norm(x_next(10:13)); % renormalize

if any(~isfinite(x_next))
    error('Nonlinear RK4 produced non-finite state.');
end
end
