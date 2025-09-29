function [A, B] = Jacobian()

    
    syms px p_y pz vx vy vz wx wy wz q0 q1 q2 q3 MT_bx MT_by MT_bz T time_elapsed Ixx Iyy Izz m

    x = [px; p_y; pz; vx; vy; vz; wx; wy; wz; q0; q1; q2; q3];
    u = [MT_bx; MT_by; MT_bz; T];

    MT_b = [MT_bx; MT_by; MT_bz];
    
    wetMOI = [0.2109 0 0; 0 21.1212 0; 0 0 21.1202];
    dryMOI = [0.2036 0 0; 0 18.99 0; 0 0 18.988];
    wetMass = 38.87;
    dryMass = 30.96;
    finalCG = 2.34;
    CG0 = 2.433;
    burnTime = 13.1;
    Ib = wetMOI - ((wetMOI - dryMOI)/burnTime) .* time_elapsed;
    inv_Ib = inv(Ib);
    m = wetMass - ((wetMass - dryMass)/burnTime) * time_elapsed; %mass
    CG = CG0 - ((CG0 - finalCG)/burnTime) * time_elapsed; %center of gravity
    rho = 0.0023769 .* (1 - 6.877e-6 .* px).^4.25588; %air density

    % had issues with complex values, chatGPT suggested this:
    % alt_ft = px * 3.280839895;           % px is meters → feet
    % base   = 1 - 6.877e-6 * alt_ft;
    % base   = max(base, 0);                  % clamp to avoid negative base
    % rho    = 0.0023769 * base^4.25588;
    %rho = 1.225 * (1 - 2.25577e-5 * px)^4.25588;

    g = 9.81;
    a = atan(vy/vx); %angle of attack
    v = sqrt(vx^2 + vy^2 + vz^2); %magnitude of velocity
    Lmag = 0.193088*rho*v^2*a / (3.59306 + sqrt(1 - (v/1097.40)^2)) + 1.22718*rho*v^2*asin(wz*(10.3264-CG)/v) / (1+sqrt(1+0.797628*(1-(v/1116.45)^2)));
    D_a = 0.46875*rho*v^2*(0.00253*a^2+0.13932);
    Mo = (a .* ((4.12421 ./ (27.81553 + 16.83430 .* sqrt(1 - (v./1097.40).^2))) + -0.00014 .* a + 0.01273)) * 0.6375*rho*v^2;
    MA_b_CG0 = [0;0;Mo];

    R = [2*(q0^2+q1^2)-1 2*(q1*q2-q0*q3) 2*(q1*q3+q0*q2); 2*(q1*q2+q0*q3) 2*(q0^2+q2^2)-1 2*(q2*q3-q0*q0); 2*(q1*q3-q0*q2) 2*(q2*q3+q0*q1) 2*(q0^2+q3^2)-1];
    inv_R = inv(R);

    % This term containts the controls (ARE THE CONTROLS PROPERLY INTEGRATED?)
    wbf_b_dot = Ib\[MT_bx + Ib(2,2)*wy*wz-Ib(3,3)*wy*wz; MT_by + Ib(3,3)*wx*wz-Ib(1,1)*wx*wz; MT_bz + Mo+Ib(1,1)*wx*wy-Ib(2,2)*wx*wy];
    
    
    % Symbolic Equations of Motion in terms of State and Control

    px_dot = vx*inv_R(1,1) + vy*inv_R(1,2) + vz*inv_R(1,3);
    py_dot = vx*inv_R(2,1) + vy*inv_R(2,2) + vz*inv_R(2,3);
    pz_dot = vx*inv_R(3,1) + vy*inv_R(3,2) + vz*inv_R(3,3);
    p_dot = [px_dot; py_dot; pz_dot];

    vx_dot = (Lmag*sin(a) - D_a*cos(a) + 0.92*T - m*g*(2*(q0^2+q1^2)-1))/m + wz*vy-wy*vz;
    vy_dot = (-Lmag*cos(a) - D_a*sin(a) - 2*m*g*(q1*q2+q0*q3)/m + wx*vz-wz*vx);
    vz_dot = -2*m*g*(q1*q3-q0*q2)/m + wy*vx-wx*vy;
    v_dot = [vx_dot; vy_dot; vz_dot];

    wx_dot = wbf_b_dot(1);
    wy_dot = wbf_b_dot(2);
    wz_dot = wbf_b_dot(3);
    w_dot = [wx_dot; wy_dot; wz_dot];

    q0_dot = 0.5 * (-wx*q1 - wy*q2 - wz*q3);
    q1_dot = 0.5 * (wx*q0 + wz*q2 - wy*q3);
    q2_dot = 0.5 * (wy*q0 - wz*q1 + wx*q3);
    q3_dot = 0.5 * (wz*q0 + wy*q1 - wx*q2);
    q_dot = [q0_dot; q1_dot; q2_dot; q3_dot];
    
    x_dot = [p_dot; v_dot; w_dot; q_dot;];

    A = jacobian(x_dot, x);
    B = jacobian(x_dot, u);

    A = real(A);
    B = real(B);



   % for potentially implicit dynamics
   % f = xDot - xDot_var;
   % 
   % E = jacobian(f, xDot);
   % A_prime = jacobian(f, x_var);
   % B_prime = jacobian(f, u);
   % 
   % A = -1*inv(E)*A_prime;
   % B = -1*inv(E)*B_prime;
   % 
   % vars = [x_var, u];
   % XU = [x0; u0];
   % A_num = double(subs(A, vars, XU));
   % B_num = double(subs(B, vars, XU));
end
   