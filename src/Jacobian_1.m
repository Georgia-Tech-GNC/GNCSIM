function [A, B] = Jacobian_1()


    % need thrust to already be constant in dynamics
    % syms px p_y pz u v w rho CG p r
    % Vars = [px p_y pz u v w rho CG p r];

    syms t T
    syms x y z u v w p q r q0 q1 q2 q3 
    syms xdot ydot zdot udot vdot wdot pdot qdot rdot q0dot q1dot q2dot q3dot
    syms u1 u2 u3 % Roll moment, pitch moment, yaw moment

    xdot_vec = [xdot ydot zdot udot, vdot, wdot, pdot, qdot, rdot, q0dot, q1dot, q2dot, q3dot];
    x_vec = [x; y; z; u; v; w; p; q; r; q0; q1; q2; q3];
    u_vec = [u1; u2; u3];


    g = 9.81; 
    wet_mass = 40.912994; % kg
    dry_mass = 32.619994;
    initial_CG = 2.419762;
    final_CG = 2.314814;
    wetRollMOI = 0.209731;  % Ixx
    wetPitchMOI = 24.655278;  % Iyy
    wetYawMOI = 24.656148;  % Izz
    dryRollMOI = 0.201846;
    dryPitchMOI = 22.301315;
    dryYawMOI = 22.301570;
   

    burn_time = 13.1;

    rho = 0.0023769 .* (1 - 6.877e-6 .* x).^4.25588;

    %R = [2*(q0^2+q1^2)-1, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2); 2*(q1*q2+q0*q3), 2*(q0^2+q2^2)-1, 2*(q2*q3-q0*q1); 2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), 2*(q0^2+q3^2)-1];
    % R = [1 - 2*(q2^2 + q3^2),     2*(q1*q2 - q0*q3),     2*(q1*q3 + q0*q2);
    %  2*(q1*q2 + q0*q3),       1 - 2*(q1^2 + q3^2),   2*(q2*q3 - q0*q1);
    %  2*(q1*q3 - q0*q2),       2*(q2*q3 + q0*q1),     1 - 2*(q1^2 + q2^2)];

    R = [ ...
    q0^2 + q1^2 - q2^2 - q3^2,   2*(q1*q2 + q0*q3),     2*(q1*q3 - q0*q2);
    2*(q1*q2 - q0*q3),           q0^2 - q1^2 + q2^2 - q3^2, 2*(q2*q3 + q0*q1);
    2*(q1*q3 + q0*q2),           2*(q2*q3 - q0*q1),     q0^2 - q1^2 - q2^2 + q3^2 ...
    ]; % inertial to body

    inv_R = inv(R); % body to inertial

    
    
   
    
   


    m  = piecewise(t < burn_time, wet_mass - ((wet_mass - dry_mass)/burn_time)*t, dry_mass);
    CG = piecewise(t < burn_time, initial_CG - ((initial_CG - final_CG)/burn_time)*t, final_CG);
    Ixx = piecewise(t < burn_time, wetRollMOI - ((wetRollMOI - dryRollMOI)/burn_time)*t, dryRollMOI);
    Iyy = piecewise(t < burn_time, wetPitchMOI - ((wetPitchMOI - dryPitchMOI)/burn_time)*t, dryPitchMOI);
    Izz = piecewise(t < burn_time, wetYawMOI - ((wetYawMOI - dryYawMOI)/burn_time)*t, dryYawMOI);

   

    rocket_params = [wet_mass, dry_mass, initial_CG, final_CG, ...
                     Ixx, dryRollMOI, Iyy, dryPitchMOI, ...
                     Izz, dryYawMOI, burn_time];

    V = sqrt(u.^2 + v.^2 + w.^2 + 1e-12);
    % V = max(V, 1e-6);         % avoid division by zero / tiny V

    % arg = r.*(10.3264 - 3.281*CG) ./ (3.281 .* V);
    % arg_clamped = max(-1, min(1, arg));
    % theta = asin(arg_clamped);
    asin_arg = r*(10.3264-(3.281*CG))/(3.281*(V));
    asin_expansion_5 = asin_arg + (asin_arg^3)/6 + 3*(asin_arg^5) / 40; 
    alpha = v/u;
    
    

    FAx_b_sym = (1/0.225)*(((0.64181 .* ((alpha)*(180/pi)) .* (0.64181 ./ (3.59306 + sqrt(1 - ((3.281*(sqrt(u^2 + v^2 + w^2)))./1097.40).^2)))) .* 0.5 .* (rho*0.00194) .* (3.281*(sqrt(u^2 + v^2 + w^2))).^2 .* 0.9375 + 0.5*(rho*0.00194)*(3.281*(sqrt(u^2 + v^2 + w^2)))^2*0.9375 * (2.61799)/(1+sqrt(1+0.797628*(1-((3.281*(sqrt(u^2 + v^2 + w^2)))/1116.45)^2))) * asin(asin_arg))*sin(alpha) + ((0.00253 * ((alpha)*(180/pi)).^2 + 0.13932) .* 0.5 .* (rho*0.00194) .* (3.281*(sqrt(u^2 + v^2 + w^2))).^2 .* 0.9375)*-1*cos(alpha));
    FAy_b_sym = (1/0.225)*(((0.64181 .* ((alpha)*(180/pi)) .* (0.64181 ./ (3.59306 + sqrt(1 - ((3.281*(sqrt(u^2 + v^2 + w^2)))./1097.40).^2)))) .* 0.5 .* (rho*0.00194) .* (3.281*(sqrt(u^2 + v^2 + w^2))).^2 .* 0.9375 + 0.5*(rho*0.00194)*(3.281*(sqrt(u^2 + v^2 + w^2)))^2*0.9375 * (2.61799)/(1+sqrt(1+0.797628*(1-((3.281*(sqrt(u^2 + v^2 + w^2)))/1116.45)^2))) * asin(asin_arg))*-1*cos(alpha) + ((0.00253 * ((alpha)*(180/pi)).^2 + 0.13932) .* 0.5 .* (rho*0.00194) .* (3.281*(sqrt(u^2 + v^2 + w^2))).^2 .* 0.9375)*-1*sin(alpha));
    FAz_b_sym = 0;
    MAx_b_sym = 1.36*((((4.*((2.*pi)./sqrt(1-((3.281*sqrt(u^2 + v^2 + w^2))./1116.4).^2)).*p)./(0.4688.*(3.281*sqrt(u^2 + v^2 + w^2)))).*0.1897).*0.5.*(rho*0.00194).*(3.281*sqrt(u^2 + v^2 + w^2)).^2.*0.4688);
    MAy_b_sym = 0;
    MAz_b_sym = 1.36*((((alpha)*(180/pi)) .* ((4.12421 ./ (27.81553 + 16.83430 .* sqrt(1 - ((3.281*sqrt(u^2 + v^2 + w^2))./1097.40).^2))) + -0.00014 .* ((alpha)*(180/pi)) + 0.01273)) * 0.5 * (rho*0.00194) * (3.281*sqrt(u^2 + v^2 + w^2)).^2 * 0.9375 + 0.5*(rho*0.00194)*(3.281*sqrt(u^2 + v^2 + w^2))^2*0.9375 * (2.61799)/(1+sqrt(1+0.797628*(1-((3.281*sqrt(u^2 + v^2 + w^2))/1116.45)^2))) * asin(r*(10.3264-(3.281*CG))/(3.281*sqrt(u^2 + v^2 + w^2))) * (10.3264-(3.281*CG)));
    
    
    

    aero_forces_moments = [FAx_b_sym, FAy_b_sym, FAz_b_sym, MAx_b_sym, MAy_b_sym, MAz_b_sym];


    EOM_no_aero = [u*inv_R(1,1) + v*inv_R(1,2) + w*inv_R(1,3) - xdot, % inv_R 
        u*inv_R(2,1) + v*inv_R(2,2) + w*inv_R(2,3) - ydot,
        u*inv_R(3,1) + v*inv_R(3,2) + w*inv_R(3,3) - zdot,
        (1/m)*(R(1,3)*m*g + 0.95*T) + r*v - q*w - udot,
        R(2,3)*g + p*w - r*u - vdot,
        R(3,3)*g + q*u - p*v - wdot,
        (1/Ixx)*(u1 + q*r*(Iyy - Izz)) - pdot,
        (1/Iyy)*(u2 + p*r*(Izz - Ixx)) - qdot,
        (1/Izz)*(u3 + p*q*(Ixx - Iyy)) - rdot,
        0.5 * (-p*q1 - q*q2 - r*q3) - q0dot,
        0.5*(p*q0 + r*q2 - q*q3) - q1dot,
        0.5*(q*q0 - r*q1 + p*q3) - q2dot,
        0.5*(r*q0 + q*q1 - p*q2) - q3dot];

    FAx = aero_forces_moments(1);
    FAy = aero_forces_moments(2);
    FAz = aero_forces_moments(3);
    MAx = aero_forces_moments(4);
    MAy = aero_forces_moments(5);
    MAz = aero_forces_moments(6);

    EOM_only_aero = [0,
                     0,
                     0,
                     (1/m)*FAx,
                     (1/m)*FAy,
                     (1/m)*FAz,
                     (1/Ixx)*MAx,
                     (1/Iyy)*MAy,
                     (1/Izz)*MAz,
                     0,
                     0,
                     0,
                     0]; 

  

   

    nonlinear_EOM = EOM_no_aero; % + EOM_only_aero; % remove second term to test w/o aero
    
    F = nonlinear_EOM;

    E = jacobian(F, xdot_vec);
    A_prime = jacobian(F, x_vec);
    B_prime = jacobian(F, u_vec);

    A = -E\A_prime;
    B = -E\B_prime;
end