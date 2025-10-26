run_time = 13.1; % simulation length

burn_time = 13.1;
delta_t = 0.1; % for uniform time interval break points
dt = .01;
steps = round(run_time / dt, 0);


x = [0; % initial state
     0;
     0;
     .001;
     0;
     0;
     0;
     0;
     0;
     1/sqrt(2);
     0;
     1/sqrt(2);
     0];



u = zeros(3,1); % initial control

% straight up Q and R (from masterfile)
Q = ones(13, 1);
R = ones(3, 1);

% need to tune: what was .01 is now 1, what was massive is now 100
Q(1) = .01; % x
Q(2) = 1000; % y
Q(3) = 1000; % z
Q(4) = .01; % u
Q(5) = .01; % v
Q(6) = .01; % w
Q(7) = .01; % p
Q(8) = .01; % q
Q(9) = .01; % r
Q(10) = .01; % q0
Q(11) = 1000; % q1
Q(12) = .01; % q2
Q(13) = .01; % q3

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
thrust_fun  = @(tt) getThrust(tt, arr, burn_time);  % table-based thrust

%% Generate Trajectory

data = struct("x",[0],"y",[0],"z",[0],"u",[0],"v",[0.01],"w",[0],"p",[0],"q",[0],"r",[0], ...
"q0",[sqrt(2)/2],"q1",[0],"q2",[sqrt(2)/2],"q3",[0], "T",[0], "roll",[0], "pitch",[0], "yaw",[0]);

% to check aero stuff
data.t   = 0;
data.FAx = 0; data.FAy = 0; data.FAz = 0;   % body forces [N]
data.MAx = 0; data.MAy = 0; data.MAz = 0;   % body moments [N·m]


for i = 1:steps

    t = dt * (i-1);
   
    currentThrust = getThrust(t, arr, burn_time);
    
    
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
    %if 1 == 2  % to only propagate, not optimize
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
        
        arg_asin = x(9) * (10.3264 - 3.281*CGt) / (3.281*V); % r*lever / (3.281*V)
        
        if M1 >= 1 || abs(arg_asin) > 1
            warning('Aero domain breach at t=%.3f: M1=%.3f, asin_arg=%.3f; Jacobian will go complex.', ...
                     t, M1, arg_asin);
        end
    
    
        % Substitute into symbolic Jacobians (A,B were built once at the top)
        A_num = subs(A, all_names, all_values);
        B_num = subs(B, all_names, all_values);
        % [A_num, B_num] = eval_AB_safe(A, B, x, u, currentThrust, t);
    
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
        x = RK4_propagate(x, u, t, dt, eom_fun, thrust_fun);
    end

    % --- INLINE aero recompute for logging (must match eom_nonlin) ---
    xi = x(1); yi = x(2); zi = x(3); 
    uB = x(4); vB = x(5); wB = x(6);
    pB = x(7); qB = x(8); rB = x(9);
    q0 = x(10); q1 = x(11); q2 = x(12); q3 = x(13); 
    
    % CG(t) like your model
    burn_time   = 13.1;
    initial_CG  = 2.419762;
    final_CG    = 2.314814;
    if t < burn_time
        CG = initial_CG - ((initial_CG - final_CG)/burn_time) * t;
    else
        CG = final_CG;
    end
    
    % IMPORTANT: your inertial is NED -> altitude ≈ -z (clamp at 0)
    alt = max(0, -zi);
    
    % Atmosphere (same law you had, but use altitude not x/north)
    rho = 0.0023769 * (1 - 6.877e-6 * alt)^4.25588;
    
    V = sqrt(uB^2 + vB^2 + wB^2 + 1e-12);
    alpha = vB / uB;   % (you preferred this over atan2)
    asin_arg = rB*(10.3264 - (3.281*CG)) / (3.281*V);

    AeroData = struct("u_in",[0.01],"v_in",[0],"w_in",[0],"rho_in",[0],"CG_in",[0],"p_in",[0],"r_in",[0], ...
        "FAx",[0],"FAy",[0],"FAz",[0],"MAx",[0],"MAy",[0],"MAz",[0]);
    

    FAx = (1/0.225)*(((0.64181 .* ((vB/uB)*(180/pi)) .* (0.64181 ./ (3.59306 + sqrt(1 - ((3.281*(sqrt(uB^2 + vB^2 + wB^2)))./1097.40).^2)))) .* 0.5 .* (rho*0.00194) .* (3.281*(sqrt(uB^2 + vB^2 + wB^2))).^2 .* 0.9375 + 0.5*(rho*0.00194)*(3.281*(sqrt(uB^2 + vB^2 + wB^2)))^2*0.9375 * (2.61799)/(1+sqrt(1+0.797628*(1-((3.281*(sqrt(uB^2 + vB^2 + wB^2)))/1116.45)^2))) * asin(rB*(10.3264-(3.281*CG))/(3.281*(sqrt(uB^2 + vB^2 + wB^2)))))*sin(vB/uB) + ((0.00253 * ((vB/uB)*(180/pi)).^2 + 0.13932) .* 0.5 .* (rho*0.00194) .* (3.281*(sqrt(uB^2 + vB^2 + wB^2))).^2 .* 0.9375)*-1*cos(vB/uB));
    FAy = (1/0.225)*(((0.64181 .* ((vB/uB)*(180/pi)) .* (0.64181 ./ (3.59306 + sqrt(1 - ((3.281*(sqrt(uB^2 + vB^2 + wB^2)))./1097.40).^2)))) .* 0.5 .* (rho*0.00194) .* (3.281*(sqrt(uB^2 + vB^2 + wB^2))).^2 .* 0.9375 + 0.5*(rho*0.00194)*(3.281*(sqrt(uB^2 + vB^2 + wB^2)))^2*0.9375 * (2.61799)/(1+sqrt(1+0.797628*(1-((3.281*(sqrt(uB^2 + vB^2 + wB^2)))/1116.45)^2))) * asin(rB*(10.3264-(3.281*CG))/(3.281*(sqrt(uB^2 + vB^2 + wB^2)))))*-1*cos(vB/uB) + ((0.00253 * ((vB/uB)*(180/pi)).^2 + 0.13932) .* 0.5 .* (rho*0.00194) .* (3.281*(sqrt(uB^2 + vB^2 + wB^2))).^2 .* 0.9375)*-1*sin(vB/uB));
    FAz = 0;
    MAx = 1.36*((((4.*((2.*pi)./sqrt(1-((3.281*sqrt(uB^2 + vB^2 + wB^2))./1116.4).^2)).*pB)./(0.4688.*(3.281*sqrt(uB^2 + vB^2 + wB^2)))).*0.1897).*0.5.*(rho*0.00194).*(3.281*sqrt(uB^2 + vB^2 + wB^2)).^2.*0.4688);
    MAy = 0;
    MAz = 1.36*((((vB/uB)*(180/pi)) .* ((4.12421 ./ (27.81553 + 16.83430 .* sqrt(1 - ((3.281*sqrt(uB^2 + vB^2 + wB^2))./1097.40).^2))) + -0.00014 .* ((vB/uB)*(180/pi)) + 0.01273)) * 0.5 * (rho*0.00194) * (3.281*sqrt(uB^2 + vB^2 + wB^2)).^2 * 0.9375 + 0.5*(rho*0.00194)*(3.281*sqrt(uB^2 + vB^2 + wB^2))^2*0.9375 * (2.61799)/(1+sqrt(1+0.797628*(1-((3.281*sqrt(uB^2 + vB^2 + wB^2))/1116.45)^2))) * asin(rB*(10.3264-(3.281*CG))/(3.281*sqrt(uB^2 + vB^2 + wB^2))) * (10.3264-(3.281*CG)));
    
    % Append to logs
    data.t = [data.t;t];
    data.FAx = [data.FAx;FAx];
    data.FAy = [data.FAy;FAy];
    data.FAz = [data.FAz;FAz];
    data.MAx = [data.MAx;MAx];
    data.MAy = [data.MAy;MAy];
    data.MAz = [data.MAz;MAz];

    % aero check log
    AeroData.u_in = [AeroData.u_in;uB];
    AeroData.v_in = [AeroData.v_in;vB];
    AeroData.w_in = [AeroData.w_in;wB];
    AeroData.rho_in = [AeroData.rho_in;rho];
    AeroData.CG_in = [AeroData.CG_in;CG];
    AeroData.p_in = [AeroData.p_in;pB];
    AeroData.r_in = [AeroData.r_in;rB];
    AeroData.FAx = [AeroData.FAx;FAx];
    AeroData.FAy = [AeroData.FAy;FAy];
    AeroData.FAz = [AeroData.FAz;FAz];
    AeroData.MAx = [AeroData.MAx;MAx];
    AeroData.MAy = [AeroData.MAy;MAy];
    AeroData.MAz = [AeroData.MAz;MAz];
    % ---------------------------------------------------

    
    
    % add to struct
    data.x = [data.x;x(1)]; 
    data.y = [data.y;x(2)];
    data.z = [data.z;x(3)]; % NED convention: z = -altitude
    data.u = [data.u;x(4)];
    data.v = [data.v;x(5)];
    data.w = [data.w;x(6)];
    data.p = [data.p;x(7)];
    data.q = [data.q;x(8)];
    data.r = [data.r;x(9)];
    data.q0 = [data.q0;x(10)];
    data.q1 = [data.q1;x(11)];
    data.q2 = [data.q2;x(12)];
    data.q3 = [data.q3;x(13)];
    
    data.roll = [data.roll;u(1)];
    data.pitch = [data.pitch;u(2)];
    data.yaw = [data.yaw;u(3)];
    %data.thrust = [data.thrust,u(4)];

end

disp("Success!")

display_results(data)



function []= display_results(data)

    figure(1); clf
    plot3(data.x, data.y, -data.z)
    xlabel('X')
    ylabel('Y')
    zlabel('Z (m)')
    title('3D Trajectory')

    figure(2); clf
    plot(data.t, atan2(data.v,data.u))
    xlabel('t (s)')
    ylabel('alpha')
    title('3D Trajectory')

    figure(2); clf
    plot(data.t, atan2(data.v,data.u))
    xlabel('t (s)')
    ylabel('alpha')
    title('Angle of Attack ')

    figure(3); clf
    plot(data.t, data.x)
    xlabel('t (s)')
    ylabel('alpha')
    title('3D Trajectory')








end
