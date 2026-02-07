function [omega_cmd, quat_f, theta_deg] = omega_for_quat_err(x, quat_goal)
% omega command from quaternion error (stable, dt-independent)
% x(10:13) = [q0 q1 q2 q3] scalar-first
% omega_cmd returned as [p;q;r] (body rates)
kp = 4.0;
kd = 1.0;
omega_max = 6;

q = x(10:13);
q = q / norm(q);
omega = x(7:9);

% shortest path
if dot(q, quat_goal) < 0
    quat_goal = -quat_goal;
end

% q_err = q^{-1} ⊗ q_goal  (error expressed in body frame)
q_conj = [ q(1); -q(2); -q(3); -q(4) ];

q_err = quatmultiply_sf(q_conj, quat_goal);   % scalar-first multiply

% axis-angle from q_err
w = q_err(1);
v = q_err(2:4);
nv = norm(v);

theta = 2*atan2(nv, max(1e-12, w));  % in [0,pi]
theta_deg = rad2deg(theta);

if nv < 1e-12
    axis = [0;0;0];
else
    axis = v / nv;
end

% PD law in body rates
omega_cmd = kp * theta * axis - kd * omega;

% clamp
om = norm(omega_cmd);
if om > omega_max
    omega_cmd = omega_cmd * (omega_max/om);
end

% "intermediate target quaternion" for logging if you want it
quat_f = quat_goal;
end

function q = quatmultiply_sf(q1,q2)
% scalar-first quaternion multiply: q = q1 ⊗ q2
w1=q1(1); x1=q1(2); y1=q1(3); z1=q1(4);
w2=q2(1); x2=q2(2); y2=q2(3); z2=q2(4);
q = [ w1*w2 - x1*x2 - y1*y2 - z1*z2;
      w1*x2 + x1*w2 + y1*z2 - z1*y2;
      w1*y2 - x1*z2 + y1*w2 + z1*x2;
      w1*z2 + x1*y2 - y1*x2 + z1*w2 ];
end
