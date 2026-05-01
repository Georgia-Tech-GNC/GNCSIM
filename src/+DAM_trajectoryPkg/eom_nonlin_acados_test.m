function xdot = eom_nonlin_acados(x, beta_vec, aero, t, T)
import casadi.*
% xdot = f(x,u_mom,t,T,aero)
% x = [x y z u v w p q r q0 q1 q2 q3]'
% u_mom = [u1 u2 u3]' (body moments)
% T = thrust magnitude (scalar) along +x_b
% aero = 0 or 1 (flag)

    % unpack
    xi = x(1); yi = x(2); zi = x(3);
    u  = x(4); v  = x(5); w  = x(6);
    p  = x(7); q  = x(8); r  = x(9);
    q0 = x(10); q1 = x(11); q2 = x(12); q3 = x(13);

    beta1a = beta_vec(1); beta1b = beta_vec(2); beta2a = beta_vec(3); beta2b = beta_vec(4);

    % constants
    g  = 9.81;
    wet_mass  = 29.04;
    dry_mass  = 24.72;
    initial_CG = 1.98;
    final_CG   = 1.9;
    burn_time  = 13.1;

    wetRollMOI  = 0.12;  % Ixx
    wetPitchMOI = 13.33; % Iyy
    wetYawMOI   = 13.33; % Izz
    dryRollMOI  = 0.11;
    dryPitchMOI = 11.95;
    dryYawMOI   = 11.95;

    wetMOI = [wetRollMOI 0 0; 0 wetPitchMOI 0; 0 0 wetYawMOI];
    dryMOI = [dryRollMOI 0 0; 0 dryPitchMOI 0; 0 0 dryYawMOI];

    % quaternion DCM 
    R = [ ...
    q0^2 + q1^2 - q2^2 - q3^2,   2*(q1*q2 + q0*q3),     2*(q1*q3 - q0*q2);
    2*(q1*q2 - q0*q3),           q0^2 - q1^2 + q2^2 - q3^2, 2*(q2*q3 + q0*q1);
    2*(q1*q3 + q0*q2),           2*(q2*q3 - q0*q1),     q0^2 - q1^2 - q2^2 + q3^2 ...
    ];

    inv_R = inv(R);

    % mass & CG vs time: CasADi-friendly piecewise
    m_wet  = wet_mass - ((wet_mass - dry_mass)/burn_time) * t;
    CG_wet = initial_CG - ((initial_CG - final_CG)/burn_time) * t;
    Ib_wet = wetMOI - ((wetMOI - dryMOI)/burn_time) .* t;

    cond_burn = (t < burn_time);

    m  = if_else(cond_burn, m_wet,  dry_mass);
    CG = if_else(cond_burn, CG_wet, final_CG);

    % Ib is 3x3 -> apply if_else elementwise
    % Make constant MOI matrices CasADi constants (optional but clean)
    wetMOI = SX(wetMOI);
    dryMOI = SX(dryMOI);
    
    % Symbolic linear interpolation (this is MX because it uses t)
    Ib_wet = wetMOI - ((wetMOI - dryMOI)/burn_time) * t;
    
    % Elementwise burn switch (both branches are MX)
    Ib = if_else(cond_burn, Ib_wet, dryMOI);

    Ixx = Ib(1,1);
    Iyy = Ib(2,2);
    Izz = Ib(3,3);

    % aero terms
    rho = 0.0023769 * (1 - 6.877e-6 * -zi)^4.25588;
    a = 1125.33;
    
    tab_angles = [0; pi; pi/2; 3*pi/2];   % 1a,1b,2a,2b
    
    tab_A = [0;0;0;0];    % tab reference areas
    tab_l = [0;0;0;0];    % reference lengths for moments
    tab_r = zeros(3,4);   % tab aerodynamic center locations from CG
    
    % local velocities at each tab
    V_CG = [u; v; w];
    
    V1a = V_CG + [q*tab_r(3,1) - r*tab_r(2,1);
                  r*tab_r(1,1) - p*tab_r(3,1);
                  p*tab_r(2,1) - q*tab_r(1,1)];
    
    V1b = V_CG + [q*tab_r(3,2) - r*tab_r(2,2);
                  r*tab_r(1,2) - p*tab_r(3,2);
                  p*tab_r(2,2) - q*tab_r(1,2)];
    
    V2a = V_CG + [q*tab_r(3,3) - r*tab_r(2,3);
                  r*tab_r(1,3) - p*tab_r(3,3);
                  p*tab_r(2,3) - q*tab_r(1,3)];
    
    V2b = V_CG + [q*tab_r(3,4) - r*tab_r(2,4);
                  r*tab_r(1,4) - p*tab_r(3,4);
                  p*tab_r(2,4) - q*tab_r(1,4)];
    
    % local speed
    V1a_mag = sqrt(V1a(1)^2 + V1a(2)^2 + V1a(3)^2);
    V1b_mag = sqrt(V1b(1)^2 + V1b(2)^2 + V1b(3)^2);
    V2a_mag = sqrt(V2a(1)^2 + V2a(2)^2 + V2a(3)^2);
    V2b_mag = sqrt(V2b(1)^2 + V2b(2)^2 + V2b(3)^2);
    
    % local Mach
    M1a = V1a_mag / a;
    M1b = V1b_mag / a;
    M2a = V2a_mag / a;
    M2b = V2b_mag / a;
    
    % local normal vectors
    n1a = [0; -sin(tab_angles(1)); cos(tab_angles(1))];
    n1b = [0; -sin(tab_angles(2)); cos(tab_angles(2))];
    n2a = [0; -sin(tab_angles(3)); cos(tab_angles(3))];
    n2b = [0; -sin(tab_angles(4)); cos(tab_angles(4))];
    
    % local axial velocities
    Vx1a = V1a(1);
    Vx1b = V1b(1);
    Vx2a = V2a(1);
    Vx2b = V2b(1);
    
    % local normal velocities
    Vn1a = n1a(1)*V1a(1) + n1a(2)*V1a(2) + n1a(3)*V1a(3);
    Vn1b = n1b(1)*V1b(1) + n1b(2)*V1b(2) + n1b(3)*V1b(3);
    Vn2a = n2a(1)*V2a(1) + n2a(2)*V2a(2) + n2a(3)*V2a(3);
    Vn2b = n2b(1)*V2b(1) + n2b(2)*V2b(2) + n2b(3)*V2b(3);
    
    % local angle of attack
    alpha1a = atan2(Vn1a, Vx1a);
    alpha1b = atan2(Vn1b, Vx1b);
    alpha2a = atan2(Vn2a, Vx2a);
    alpha2b = atan2(Vn2b, Vx2b);
    
    % individual forces and moments
    [CN1a,CA1a,CMR1a,CMP1a] = aero_coeffs(M1a, alpha1a, beta1a);
    FA1a = 0.5 * rho * V1a_mag^2 * tab_A(1) * ...
          (-CA1a*[1;0;0] + CN1a*[0; -sin(tab_angles(1)); cos(tab_angles(1))]);
    
    MA1a = [tab_r(2,1)*FA1a(3) - tab_r(3,1)*FA1a(2);
            tab_r(3,1)*FA1a(1) - tab_r(1,1)*FA1a(3);
            tab_r(1,1)*FA1a(2) - tab_r(2,1)*FA1a(1)] ...
          + 0.5 * rho * V1a_mag^2 * tab_A(1) * tab_l(1) * ...
            (CMR1a*[1;0;0] + CMP1a*[0; cos(tab_angles(1)); sin(tab_angles(1))]);
    
    [CN1b,CA1b,CMR1b,CMP1b] = aero_coeffs(M1b, alpha1b, beta1b);
    FA1b = 0.5 * rho * V1b_mag^2 * tab_A(2) * ...
          (-CA1b*[1;0;0] + CN1b*[0; -sin(tab_angles(2)); cos(tab_angles(2))]);
    
    MA1b = [tab_r(2,2)*FA1b(3) - tab_r(3,2)*FA1b(2);
            tab_r(3,2)*FA1b(1) - tab_r(1,2)*FA1b(3);
            tab_r(1,2)*FA1b(2) - tab_r(2,2)*FA1b(1)] ...
          + 0.5 * rho * V1b_mag^2 * tab_A(2) * tab_l(2) * ...
            (CMR1b*[1;0;0] + CMP1b*[0; cos(tab_angles(2)); sin(tab_angles(2))]);
    
    [CN2a,CA2a,CMR2a,CMP2a] = aero_coeffs(M2a, alpha2a, beta2a);
    FA2a = 0.5 * rho * V2a_mag^2 * tab_A(3) * ...
          (-CA2a*[1;0;0] + CN2a*[0; -sin(tab_angles(3)); cos(tab_angles(3))]);
    
    MA2a = [tab_r(2,3)*FA2a(3) - tab_r(3,3)*FA2a(2);
            tab_r(3,3)*FA2a(1) - tab_r(1,3)*FA2a(3);
            tab_r(1,3)*FA2a(2) - tab_r(2,3)*FA2a(1)] ...
          + 0.5 * rho * V2a_mag^2 * tab_A(3) * tab_l(3) * ...
            (CMR2a*[1;0;0] + CMP2a*[0; cos(tab_angles(3)); sin(tab_angles(3))]);
    
    [CN2b,CA2b,CMR2b,CMP2b] = aero_coeffs(M2b, alpha2b, beta2b);
    FA2b = 0.5 * rho * V2b_mag^2 * tab_A(4) * ...
          (-CA2b*[1;0;0] + CN2b*[0; -sin(tab_angles(4)); cos(tab_angles(4))]);
    
    MA2b = [tab_r(2,4)*FA2b(3) - tab_r(3,4)*FA2b(2);
            tab_r(3,4)*FA2b(1) - tab_r(1,4)*FA2b(3);
            tab_r(1,4)*FA2b(2) - tab_r(2,4)*FA2b(1)] ...
          + 0.5 * rho * V2b_mag^2 * tab_A(4) * tab_l(4) * ...
            (CMR2b*[1;0;0] + CMP2b*[0; cos(tab_angles(4)); sin(tab_angles(4))]);
    
    % total body-frame tab forces and moments
    FAx = FA1a(1) + FA1b(1) + FA2a(1) + FA2b(1);
    FAy = FA1a(2) + FA1b(2) + FA2a(2) + FA2b(2);
    FAz = FA1a(3) + FA1b(3) + FA2a(3) + FA2b(3);
    
    MAx = MA1a(1) + MA1b(1) + MA2a(1) + MA2b(1);
    MAy = MA1a(2) + MA1b(2) + MA2a(2) + MA2b(2);
    MAz = MA1a(3) + MA1b(3) + MA2a(3) + MA2b(3);

    if ~aero
        FAx = 0; FAy = 0; FAz = 0;
        MAx = 0; MAy = 0; MAz = 0;
    end

    % --------------------
    % 6-DOF Dynamics 
    % --------------------
    xdot_x = u*inv_R(1,1) + v*inv_R(1,2) + w*inv_R(1,3);
    xdot_y = u*inv_R(2,1) + v*inv_R(2,2) + w*inv_R(2,3);
    xdot_z = u*inv_R(3,1) + v*inv_R(3,2) + w*inv_R(3,3);

    % --------------------
    % Translational dynamics in body axes 
    % --------------------
    udot = (1/m)*( R(1,3)*m*g + 0.95*T + FAx ) + r*v - q*w;
    vdot = (1/m)*( FAy ) + p*w - r*u + R(2,3)*g;
    wdot = (1/m)*( FAz ) + q*u - p*v + R(3,3)*g;

    % --------------------
    % Rotational dynamics 
    % --------------------
    pdot = (1/Ixx)*(u1 + MAx + q*r*(Iyy - Izz));
    qdot = (1/Iyy)*(u2 + MAy + p*r*(Izz - Ixx));
    rdot = (1/Izz)*(u3 + MAz + p*q*(Ixx - Iyy));

    % --------------------
    % Quaternion kinematics 
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
