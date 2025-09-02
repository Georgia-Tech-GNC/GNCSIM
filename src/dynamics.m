function x_f = dynamics(x0, curr_time, F_T, F_A)
% DYNAMICS  13-state flat-Earth rocket EOM (z-up), returns xdot.
% State (your order):
%   [x y z vx vy vz wx wy wz q0 q1 q2 q3]
% Inputs (body frame):
%   F_T : [3x1] thrust force
%   F_A : [3x1] aerodynamic force
%
% Notes:
% - Moments default to zero (see Mb below). If you later have thrust/aero
%   moments, extend the signature or set them via a params block.
% - Gravity is z-up (negative in z): [0;0;-g]

% --- Parameters (set these to your vehicle) ---
m = 1.0;                        % kg (replace with your mass)
J = diag([0.01, 0.01, 0.02]);   % kg*m^2 inertia in body frame (replace)
g = 9.80665;                    % m/s^2

% Default moments to zero (extend if you have them)
Mb_T = [0;0;0];
Mb_A = [0;0;0];

% --- Unpack state (use MATLAB indexing: x0(1), not x0[1]) ---
x  = x0(1);  y  = x0(2);  z  = x0(3);      
vx = x0(4);  vy = x0(5);  vz = x0(6);
wx = x0(7);  wy = x0(8);  wz = x0(9);
q0 = x0(10); q1 = x0(11); q2 = x0(12); q3 = x0(13);

% Vectors
v       = [vx; vy; vz];           % inertial velocity
omega_b = [wx; wy; wz];           % body angular velocity
q       = [q0; q1; q2; q3];       % quaternion (scalar-first)

% Net forces/moments in body frame
Fb = F_T + F_A;                   % total body force
Mb = Mb_T + Mb_A;                 % total body moment

% Normalize quaternion to avoid drift in rotation computation
nq = norm(q);
if nq > 0
    qn = q / nq;
else
    qn = [1;0;0;0]; % fallback
end

% Rotation matrix body->inertial from quaternion
R_ib = rotmat_body_to_inertial_from_q(qn);

% --- EOM ---

% (1) Translational kinematics
rdot = v;

% (2) Translational dynamics (z-up)
gi   = [0; 0; -g];
vdot = (1/m) * (R_ib * Fb) + gi;

% (3) Rotational dynamics (body frame)
omega_dot = J \ ( Mb - cross(omega_b, J*omega_b) );

% (4) Quaternion kinematics (scalar-first)
qdot = 0.5 * quat_kinematics_matrix(omega_b) * q;

% Pack derivative in your order
x_f = [
    rdot;        % xdot, ydot, zdot
    vdot;        % vxdot, vydot, vzdot
    omega_dot;   % wxdot, wydot, wzdot
    qdot         % q0dot, q1dot, q2dot, q3dot
];

% (Optional) If you prefer to keep q normalized *within* this function,
% uncomment the next 2 lines (but usually normalize after integration step):
% qn_next = (q + qdot*0) / norm(q + qdot*0); 
% x_f(10:13) = qdot;

end

% ----------------- Helpers -----------------

function R = rotmat_body_to_inertial_from_q(q)
% q = [q0; q1; q2; q3], scalar-first, unit quaternion
q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
q0q0=q0*q0; q1q1=q1*q1; q2q2=q2*q2; q3q3=q3*q3;
q0q1=q0*q1; q0q2=q0*q2; q0q3=q0*q3; q1q2=q1*q2; q1q3=q1*q3; q2q3=q2*q3;

R = [ ...
  1-2*(q2q2+q3q3),   2*(q1q2 - q0q3),   2*(q1q3 + q0q2);
    2*(q1q2 + q0q3), 1-2*(q1q1+q3q3),   2*(q2q3 - q0q1);
    2*(q1q3 - q0q2),   2*(q2q3 + q0q1), 1-2*(q1q1+q2q2) ];
end

function Om = quat_kinematics_matrix(omega_b)
% Omega(omega_b) for scalar-first quaternion kinematics: qdot = 0.5 * Om * q
wx = omega_b(1); wy = omega_b(2); wz = omega_b(3);
Om = [  0,  -wx,  -wy,  -wz;
       wx,    0,   wz,  -wy;
       wy,  -wz,    0,   wx;
       wz,   wy,  -wx,    0 ];
end
