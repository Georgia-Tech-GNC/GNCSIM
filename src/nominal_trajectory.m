
burn_time = 13.1;
delta_t = 0.1; % for uniform time interval break points
dt = .01;
steps = round(burn_time / dt, 0);

x = [0;
     0;
     0;
     .001;
     0;
     0;
     0;
     0;
     0;
     1;
     0;
     0;
     0];

u = zeros(3,1); % initial control

% straight up Q and R (from masterfile)
Q = ones(13, 1);
R = ones(3, 1);

% need to tune: what was .01 is now 1, what was massive is now 100
Q(1) = 1; % px
Q(2) = 1000; % p_y
Q(3) = 1000; % pz
Q(4) = 1; % u
Q(5) = 1; % v
Q(6) = 1; % w
Q(7) = 1; % p
Q(8) = 1; % q
Q(9) = 1; % r
Q(10) = 1; % q0
Q(11) = 1000; % q1
Q(12) = 1; % q2
Q(13) = 1; % q3

R(1) = 1; % M_roll
R(2) = 1000; % M_pitch
R(3) = 1; % M_yaw
%R(4) = 1000; % Thrust

Q = diag(1*Q);
R = diag(10*R);

[A,B] = Jacobian_1(); % Symbolic Jacobian matrices in x_dot = Ax + Bu

u_max = 80; % maximum magnitude of u

% thrust curve
arr = [0.0, 0.457288;
0.152, 0.314574;
0.162, 40.2069;
0.164, 148.3954;
0.168, 598.904;
0.172, 890.058;
0.176, 995.096;
0.178, 1158.031;
0.182, 1342.306;
0.19, 1556.716;
0.222, 1959.8;
0.234, 2050.53;
0.254, 2111.01;
0.308, 2119.9;
0.356, 2073.29;
0.47, 2091.07;
0.528, 1989.543;
0.582, 1825.825;
0.648, 1733.526;
0.782, 1658.07;
0.892, 1701.725;
0.914, 1633.231;
0.966, 1669.558;
1.7839, 1594.076;
1.8239, 1551.156;
3.1999, 1533.43;
5.4538, 1378.986;
6.9577, 1170.933;
10.9216, 424.692;
13.5895, 106.0499;
14.6754, 12.60914;
16.1354, 0.0;];

eom_fun     = @eom_nonlin;                          % your nonlinear RHS
thrust_fun  = @(tt) thrust_at(tt, arr, burn_time);  % table-based thrust

%% Generate Trajectory

data = struct("px",[0],"p_y",[0],"pz",[0],"vx",[0],"vy",[0],"vz",[0],"wx",[0],"wy",[0],"wz",[0], ...
"q0",[1],"q1",[0],"q2",[0],"q3",[0], "T",[0], "roll",[0], "pitch",[0], "yaw",[0]);

for i = 1:steps

t = dt * (i-1);




% currentThrust function from Rocket.m
currentThrust = 0;
lowerBoundTime   = [];
upperBoundTime   = [];
lowerBoundThrust = [];
upperBoundThrust = [];
skipInterpolation = false;
found = false;
j = 1;

% quick outs
n = size(arr,1);
if t > burn_time
    currentThrust = 0;
    skipInterpolation = true;   % nothing to interpolate after burnout
elseif t <= arr(1,1)
    currentThrust = arr(1,2);   % before/at first sample, clamp
    skipInterpolation = true;
elseif t >= arr(n,1)
    currentThrust = arr(n,2);   % after/at last sample, clamp
    skipInterpolation = true;
end

% while loop that outputs values to be linearily interpolated
while (j < n) && ~found && ~skipInterpolation
    t0 = arr(j,   1);
    t1 = arr(j+1, 1);
    y0 = arr(j,   2);
    y1 = arr(j+1, 2);

    if t0 == t
        skipInterpolation = true;
        currentThrust = y0;
        found = true;
    elseif (t0 < t) && (t < t1)
        lowerBoundTime   = t0;
        upperBoundTime   = t1;
        lowerBoundThrust = y0;
        upperBoundThrust = y1;
        found = true;
    else
        j = j + 1;
    end
end

% linear interpolation for thrust
if ~skipInterpolation && found
deltaThrust = upperBoundThrust - lowerBoundThrust;
deltat      = upperBoundTime   - lowerBoundTime;
slope = deltaThrust / deltat;
b     = upperBoundThrust - (slope * upperBoundTime);
currentThrust = (slope * t) + b;
if ~isfinite(currentThrust)
error('currentThrust is non-finite at t=%.4f', t);
end
end


% syms px p_y pz vx vy vz wx wy wz q0 q1 q2 q3 MT_bx MT_by MT_bz T time_elapsed
% 
% % Evaluate A,B at current state and control (thx ChatGPT)
% % Collect them into one vector
% vars = [px; p_y; pz; vx; vy; vz; wx; wy; wz; q0; q1; q2; q3; ...
%     MT_bx; MT_by; MT_bz; T; ...
%     time_elapsed];
% 
% % Concatenate state and control into a single vector
% XU = [x; u; t];



%syms x y z u v w p q r q0 q1 q2 q3 u1 u2 u3 T time_elapsed

% if at break point
if mod(i-1, round(delta_t/dt)) == 0 %&& t ~= 0 % is t = 0 a breakpoint?
    % Names present in the symbolic A,B from Jacobian_1()
    state_names   = {'x','y','z','u','v','w','p','q','r','q0','q1','q2','q3'};
    control_names = {'u1','u2','u3'};
    param_names   = {'T','t'};
    all_names     = [state_names, control_names, param_names];

    % Numeric values in the same order
    all_values = num2cell([x; u; currentThrust; t].');  % 1x18 cell

    % ---- Diagnostics for domain violations ----
    V  = norm(x(4:6));                 % speed magnitude
    M1 = (3.281*V)/1116.45;            % Mach-ish factor used in your fits
    % Rebuild CG(t) numerically same as model (linear ramp):
    CGt = 2.419762 - ((2.419762 - 2.314814)/13.1)*t;
    
    arg_asin = x(9) * (10.3264 - 3.281*CGt) / (3.281*max(V,1e-9)); % r*lever / (3.281*V)
    
    if M1 >= 1 || abs(arg_asin) > 1
        warning('Aero domain breach at t=%.3f: M1=%.3f, asin_arg=%.3f; Jacobian will go complex.', ...
                 t, M1, arg_asin);
    end


    % Substitute into symbolic Jacobians (A,B were built once at the top)
    % A_num = subs(A, all_names, all_values);
    % B_num = subs(B, all_names, all_values);
    [A_num, B_num] = eval_AB_safe(A, B, x, u, currentThrust, t);

    % Convert to double and clean tiny imaginary parts
    A_num = double(A_num);
    B_num = double(B_num);

    a_imag = imag(A_num); b_imag = imag(B_num);
    [ia,ja] = find(abs(a_imag) == max(abs(a_imag( :))),1);
    [ib,jb] = find(abs(b_imag) == max(abs(b_imag(:))),1);
    if max(abs(a_imag(:))) > 5e-4
        fprintf('Max imag A at (%d,%d): %.3e\n', ia, ja, a_imag(ia,ja));
    end
    if max(abs(b_imag(:))) > 5e-4
        fprintf('Max imag B at (%d,%d): %.3e\n', ib, jb, b_imag(ib,jb));
    end

    tol_im = 5e-4;
    a_imax = max(abs(imag(A_num(:))));
    b_imax = max(abs(imag(B_num(:))));
    if a_imax > tol_im, error('A_num has significant imaginary part (max=%.3e).', a_imax); else, A_num = real(A_num); end
    if b_imax > tol_im, error('B_num has significant imaginary part (max=%.3e).', b_imax); else, B_num = real(B_num); end

    % Sanity checks
    assert(all(size(A_num)==[13 13]), 'A_num must be 13x13');
    assert(all(size(B_num)==[13  3]), 'B_num must be 13x3');
    assert(all(size(x)==[13 1]),      'x must be 13x1');
    assert(all(size(u)==[ 3 1]),      'u must be 3x1');
    if any(~isfinite(x)),      error('x has non-finite entries'); end
    if any(~isfinite(u)),      error('u has non-finite entries'); end
    if any(~isfinite(A_num(:))), error('A_num has non-finite entries'); end
    if any(~isfinite(B_num(:))), error('B_num has non-finite entries'); end

    % Optimize & propagate (thrust is a parameter, not a decision)
    [x, u] = optimize_propagate(x, Q, R, A_num, B_num, dt, u_max, eom_fun, thrust_fun, t);

else
    N = round(delta_t/dt);
    for k = 1:N
        tau = t + (k-1)*dt;  % global time at the start of this substep
        x   = RK4_propagate(x, u, tau, dt, eom_fun, thrust_fun);
    end
end


% add to struct
data.px = [data.px,x(1)];
data.p_y = [data.p_y,x(2)];
data.pz = [data.pz,x(3)];
data.vx = [data.vx,x(4)];
data.vy = [data.vy,x(5)];
data.vz = [data.vz,x(6)];
data.wx = [data.wx,x(7)];
data.wy = [data.wy,x(8)];
data.wz = [data.wz,x(9)];
data.q0 = [data.q0,x(10)];
data.q1 = [data.q1,x(11)];
data.q2 = [data.q2,x(12)];
data.q3 = [data.q3,x(13)];

data.roll = [data.roll,u(1)];
data.pitch = [data.pitch,u(2)];
data.yaw = [data.yaw,u(3)];
%data.thrust = [data.thrust,u(4)];

end

disp("Successfully generated nominal trajectory!")
