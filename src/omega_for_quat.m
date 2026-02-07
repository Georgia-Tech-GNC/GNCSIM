function [omega, quat_f] = omega_for_quat(x,quat_goal,alpha,error_angle, dt)

% finds angular velocities that will drive quaternion states to quat_goal



x(10:13) = x(10:13) / norm(x(10:13)); % normalize just in case
quat_0 = x(10:13); % initial quat

G = [-x(11), -x(12), -x(13); 
    x(10), -x(13), x(12);
    x(13), x(10), -x(11);
    -x(12), x(11), x(10)]; % quaternion matrix for solving for optimal p,q,r







if dot(quat_0,quat_goal) < 0 
    quat_goal = -quat_goal; % something about positive and negative being the same thing idk
end


cosHalf = min(1, max(-1, dot(quat_0, quat_goal)));
theta = 2*acos(cosHalf);
theta_deg = rad2deg(theta); % angle from goal attitude

if theta_deg < error_angle
    alpha = 1;
end


% omega_current = x(7:9);   % [p;q;r]
% kd = 0.8;                 % tune
% 
% if theta_deg < error_angle
%     quat_f = quat_goal;           % or quat_0; either is fine here
%     omega  = -kd * omega_current; % damp rates -> attitude holds
%     return
% end



quat_f = (1-alpha) * quat_0 + alpha * quat_goal; % linear combination of current and target quat
quat_f = quat_f / norm(quat_f); % normalize just in case


quat_dot_des = (quat_f - quat_0)/dt;
quat_dot_des = quat_dot_des - quat_0 * (quat_0.'*quat_dot_des); % from ChatGPT

omega = 2 * (G\quat_dot_des); % least squares solution to quat_dot_des = G * omega

