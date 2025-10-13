function xdot = eom_nonlin(x, u_mom, t, T)
% xdot = f(x,u_mom,t,T)
% x = [x y z u v w p q r q0 q1 q2 q3]'
% u_mom = [u1 u2 u3]' (body moments)
% T = thrust magnitude (scalar) along +x_b

    % --- unpack ---
    xi = x(1); yi = x(2); zi = x(3); 
    u  = x(4); v  = x(5); w  = x(6);
    p  = x(7); q  = x(8); r  = x(9);
    q0 = x(10); q1 = x(11); q2 = x(12); q3 = x(13);

    u1 = u_mom(1); u2 = u_mom(2); u3 = u_mom(3);

    % --- constants (as in your script) ---
    g  = 9.81;
    wet_mass  = 40.912994;
    dry_mass  = 32.619994;
    initial_CG = 2.419762;
    final_CG   = 2.314814;
    burn_time  = 13.1;

    wetRollMOI = 0.209731;  % Ixx
    wetPitchMOI = 24.655278;  % Iyy
    wetYawMOI = 24.656148;  % Izz
    dryRollMOI = 0.201846;
    dryPitchMOI = 22.301315;
    dryYawMOI = 22.301570;
    
    wetMOI = [wetRollMOI 0 0; 0 wetPitchMOI 0; 0 0 wetYawMOI];
    dryMOI = [dryRollMOI 0 0; 0 dryPitchMOI 0; 0 0 dryYawMOI];

    % atmosphere via altitude x (your x is altitude)
    rho = 0.0023769 * (1 - 6.877e-6 * xi)^4.25588;

    % --- quaternion DCM exactly as in your Jacobian_1 (the 1st version you kept) ---
    R = [ ...
    q0^2 + q1^2 - q2^2 - q3^2,   2*(q1*q2 + q0*q3),     2*(q1*q3 - q0*q2);
    2*(q1*q2 - q0*q3),           q0^2 - q1^2 + q2^2 - q3^2, 2*(q2*q3 + q0*q1);
    2*(q1*q3 + q0*q2),           2*(q2*q3 - q0*q1),     q0^2 - q1^2 - q2^2 + q3^2 ...
    ];

    inv_R = inv(R);
    

    % --- mass & CG vs time: identical piecewise ---
    if t < burnTime
        m = wetMass - ((wetMass - dryMass)/burnTime) * t;
        CG = CG0 - ((CG0 - finalCG)/burnTime) * t;
        Ib = wetMOI - ((wetMOI - dryMOI)/burnTime) .* t;
    else
        m = dryMass;
        CG = finalCG;
        Ib = dryMOI;
    end

    Ixx = Ib(1,1);
    Iyy = Ib(2,2);
    Izz = Ib(3,3);

    % --- aero terms: same formulas you use (alpha = v/u) ---
    V = sqrt(u^2 + v^2 + w^2 + 1e-12);
    alpha = v / u;                       % you said this ran longer than atan2
    asin_arg = r*(10.3264 - (3.281*CG)) / (3.281*V);
    
    FAx_b_sym = (1/0.225)*(((0.64181 .* ((alpha)*(180/pi)) .* (0.64181 ./ (3.59306 + sqrt(1 - ((3.281*(sqrt(u^2 + v^2 + w^2)))./1097.40).^2)))) .* 0.5 .* (rho*0.00194) .* (3.281*(sqrt(u^2 + v^2 + w^2))).^2 .* 0.9375 + 0.5*(rho*0.00194)*(3.281*(sqrt(u^2 + v^2 + w^2)))^2*0.9375 * (2.61799)/(1+sqrt(1+0.797628*(1-((3.281*(sqrt(u^2 + v^2 + w^2)))/1116.45)^2))) * asin(asin_arg))*sin(alpha) + ((0.00253 * ((alpha)*(180/pi)).^2 + 0.13932) .* 0.5 .* (rho*0.00194) .* (3.281*(sqrt(u^2 + v^2 + w^2))).^2 .* 0.9375)*-1*cos(alpha));
    FAy_b_sym = (1/0.225)*(((0.64181 .* ((alpha)*(180/pi)) .* (0.64181 ./ (3.59306 + sqrt(1 - ((3.281*(sqrt(u^2 + v^2 + w^2)))./1097.40).^2)))) .* 0.5 .* (rho*0.00194) .* (3.281*(sqrt(u^2 + v^2 + w^2))).^2 .* 0.9375 + 0.5*(rho*0.00194)*(3.281*(sqrt(u^2 + v^2 + w^2)))^2*0.9375 * (2.61799)/(1+sqrt(1+0.797628*(1-((3.281*(sqrt(u^2 + v^2 + w^2)))/1116.45)^2))) * asin(asin_arg))*-1*cos(alpha) + ((0.00253 * ((alpha)*(180/pi)).^2 + 0.13932) .* 0.5 .* (rho*0.00194) .* (3.281*(sqrt(u^2 + v^2 + w^2))).^2 .* 0.9375)*-1*sin(alpha));
    FAz_b_sym = 0;
    MAx_b_sym = 1.36*((((4.*((2.*pi)./sqrt(1-((3.281*sqrt(u^2 + v^2 + w^2))./1116.4).^2)).*p)./(0.4688.*(3.281*sqrt(u^2 + v^2 + w^2)))).*0.1897).*0.5.*(rho*0.00194).*(3.281*sqrt(u^2 + v^2 + w^2)).^2.*0.4688);
    MAy_b_sym = 0;
    MAz_b_sym = 1.36*((((alpha)*(180/pi)) .* ((4.12421 ./ (27.81553 + 16.83430 .* sqrt(1 - ((3.281*sqrt(u^2 + v^2 + w^2))./1097.40).^2))) + -0.00014 .* ((alpha)*(180/pi)) + 0.01273)) * 0.5 * (rho*0.00194) * (3.281*sqrt(u^2 + v^2 + w^2)).^2 * 0.9375 + 0.5*(rho*0.00194)*(3.281*sqrt(u^2 + v^2 + w^2))^2*0.9375 * (2.61799)/(1+sqrt(1+0.797628*(1-((3.281*sqrt(u^2 + v^2 + w^2))/1116.45)^2))) * asin(r*(10.3264-(3.281*CG))/(3.281*sqrt(u^2 + v^2 + w^2))) * (10.3264-(3.281*CG)));
    
    
    
    % shorthand
    FAx = FAx_b_sym; FAy = FAy_b_sym; FAz = FAz_b_sym;
    MAx = MAx_b_sym; MAy = MAy_b_sym; MAz = MAz_b_sym;
    
    % no aero
    % FAx = 0;
    % FAy = 0;
    % FAz = 0;
    
    % --------------------
    % Kinematics (match your use of inv_R)
    % --------------------
    xdot_x = u*inv_R(1,1) + v*inv_R(1,2) + w*inv_R(1,3);
    xdot_y = u*inv_R(2,1) + v*inv_R(2,2) + w*inv_R(2,3);
    xdot_z = u*inv_R(3,1) + v*inv_R(3,2) + w*inv_R(3,3);
    
    % --------------------
    % Translational dynamics in body axes (exactly your terms)
    % --------------------
    udot = (1/m)*( R(1,3)*m*g + 0.95*T + FAx ) + r*v - q*w;
    vdot = (1/m)*( FAy ) + p*w - r*u + R(2,3)*g;
    wdot = (1/m)*( FAz ) + q*u - p*v + R(3,3)*g;
    
    % --------------------
    % Rotational dynamics (your MOIs and cross-coupling)
    % --------------------
    pdot = (1/Ixx)*(u1 + MAx + q*r*(Iyy - Izz));
    qdot = (1/Iyy)*(u2 + MAy + p*r*(Izz - Ixx));
    rdot = (1/Izz)*(u3 + MAz + p*q*(Ixx - Iyy));
    
    % --------------------
    % Quaternion kinematics (scalar-first), as in your equations
    % --------------------
    q0dot = 0.5 * (-p*q1 - q*q2 - r*q3);
    q1dot = 0.5 * ( p*q0 + r*q2 - q*q3);
    q2dot = 0.5 * ( q*q0 - r*q1 + p*q3);
    q3dot = 0.5 * ( r*q0 + q*q1 - p*q2);
    
    % --------------------
    % Pack derivative
    % --------------------
    xdot = [ xdot_x;
             xdot_y;
             xdot_z;
             udot;
             vdot;
             wdot;
             pdot;
             qdot;
             rdot;
             q0dot;
             q1dot;
             q2dot;
             q3dot ];
end
