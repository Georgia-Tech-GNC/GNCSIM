% Main Authors: Aditya Swami (aswami8@gatech.edu)

classdef ParachuteDynamics < handle
    % Parachute dynamics model based on: "Parachute dynamics and perturbation analysis of precision airdrop system"
    % https://www.sciencedirect.com/science/article/pii/S1000936116300218
    
    properties
        % Parachute physical properties
        m_c         % Canopy mass (kg)
        I_p         % Parachute inertia
        L_p         % Vector from attachment to canopy CG (parachute frame) (m)
        S_p         % Parachute reference area (m^2)
        D_c         % Canopy diameter (m)
        L_s         % Suspension Line Length (m)
        deployAltitude % Altitude the Chute is Deployed At (m)
                
        % Added mass coefficients (from paper Eq. 1-2)
        a11, a33, a44, a66
        
        % Aerodynamic coefficient functions
        C_Dp_func   % Parachute drag coefficient: C_D(alpha)
        C_mp_func   % Parachute moment coefficient: C_m(alpha)
        
        % Environment
        rho         % Air density (kg/m^3)
        g           % Gravitational acceleration (m/s^2)
        
        % Parachute state
        q_p         % Quaternion: inertial frame to parachute frame [q0; q1; q2; q3]
        omega_p     % Parachute angular velocity in parachute frame (rad/s)
    end
    
    methods
        function obj = ParachuteDynamics(D_c, m_c, L_s, deployAltitude)
            % Inputs:
            %   D_c - Canopy diameter (m)
            %   m_c - Canopy mass (kg)
            %   L_s - Suspension Line Length (m)
            
            % Set basic parameters
            obj.D_c = D_c;
            obj.m_c = m_c;
            obj.rho = 1.225;
            obj.g = 9.81;
            obj.L_s = L_s;
            
            % Reference area
            obj.S_p = pi * (D_c/2)^2;
            
            % Vector from attachment to canopy CG in parachute frame
            obj.L_p = [0; 0; obj.L_s/2];
            
            % Parachute inertia NEED TO CHANGE
            r = D_c/2;
            obj.I_p = [m_c * r^2 / 4; 
                       m_c * r^2 / 2; 
                       m_c * r^2 / 2];
            
            % Added mass coefficients NEED TO CHANGE
            obj.a11 = 0.66 * m_c;  % Axial added mass
            obj.a33 = 0.66 * m_c;  % Lateral added mass
            obj.a44 = 0.055 * m_c * r^2;  % Roll added inertia
            obj.a66 = 0.055 * m_c * r^2;  % Pitch/yaw added inertia
            
            % Initialize aerodynamic coefficients from paper
            obj.initAeroCoefficients();
            
            % Initial parachute orientation
            obj.q_p = [1; 0; 0; 0];
            obj.omega_p = [0; 0; 0];
            obj.deployAltitude = deployAltitude;
        end
        
        function initAeroCoefficients(obj)
            % Aerodynamic coefficient functions from paper Fig. 11

            obj.C_Dp_func = @(alpha) 0.62 + 0.3*sin(alpha)^2;
            obj.C_mp_func = @(alpha) 0.15*sin(2*alpha);
        end
        
        function [F_rocket_com, M_rocket_com] = getForcesAndMoments(obj, ...
                v_com_earth, omega_com_earth, q_rocket, dt)
            % Compute forces and moments from parachute system
            %
            % Inputs:
            %   v_com_earth - Rocket COM velocity in earth frame
            %   omega_com_earth - Rocket angular velocity in earth frame
            %   q_rocket - Rocket quaternion: earth to rocket [q0; q1; q2; q3]
            %   dt - Time step
            %
            % Outputs:
            %   F_rocket_com - Total force in earth frame
            %   M_rocket_com - Total moment about rocket COM in earth frame
            
            % Get Attachment point from rocketCOM in rocket frame
            r_attach_rocket = [obj.L_s ; 0; 0];

            % Normalize quaternions
            q_rocket = q_rocket / norm(q_rocket);
            obj.q_p = obj.q_p / norm(obj.q_p);
            
            % Rotation matrices
            R_rocket = quatToRotMat(q_rocket);  % Rocket to earth
            R_pe = quatToRotMat(obj.q_p);       % Parachute to earth
            
            % Join position in earth frame
            r_attach_earth = R_rocket * r_attach_rocket;
            
            % Join velocity in earth frame
            v_attach_earth = v_com_earth + cross(omega_com_earth, r_attach_earth);
            
            % Relative velocity in parachute frame
            v_rel_par = R_pe' * v_attach_earth;
            V_mag = norm(v_rel_par);
            
            % Angle of Attack between velocity and parachute z-axis 
            if V_mag > 1e-6
                alpha_p = atan2(sqrt(v_rel_par(1)^2 + v_rel_par(2)^2), -v_rel_par(3));
            else
                alpha_p = 0;
            end
            
            % Aero Forces in Parachute Frame
            [F_aero_par, M_aero_par] = obj.getParachuteForces(v_rel_par, alpha_p);
            
            % Transform to earth frame
            F_aero_earth = R_pe * F_aero_par;
            M_aero_earth = R_pe * M_aero_par;
            
            % Include parachute weight
            m_total = obj.m_c;
            F_gravity_earth = [0; 0; -m_total * obj.g];
            
            % Total Forces At Join
            F_attach_earth = F_aero_earth + F_gravity_earth;
            M_attach_earth = M_aero_earth;
           
            % Force at COM (Assuming Rigid Line)
            F_rocket_com = F_attach_earth;
            
            % Moment about COM
            M_rocket_com = M_attach_earth + cross(r_attach_earth, F_attach_earth);
            
            % Update Parachute Orientation
            obj.updateParachuteDynamics(M_aero_par, R_pe, dt);
        end
        
        function [F_aero, M_aero] = getParachuteForces(obj, v_par, alpha)
            % Compute aerodynamic forces and moments in parachute frame
            %
            % Inputs:
            %   v_par - Velocity in parachute frame
            %   alpha - Angle of attack (rad)
            %
            % Outputs:
            %   F_aero - Aerodynamic force in parachute frame
            %   M_aero - Aerodynamic moment in parachute frame
            
            V_mag = norm(v_par);
            
            % Dynamic pressure
            q_dyn = 0.5 * obj.rho * V_mag^2;
            
            % Drag coefficient
            C_D = obj.C_Dp_func(alpha);
            
            % Drag force
            if V_mag > 1e-6
                F_aero = -q_dyn * obj.S_p * C_D * (v_par / V_mag);
            else
                F_aero = [0; 0; 0];
            end
            
            % Pitching moment coefficient
            C_m = obj.C_mp_func(alpha);
            
            % Moment (about y-axis in parachute frame)
            M_aero = [0; 
                      q_dyn * obj.S_p * obj.D_c * C_m; 
                      0];
        end
        
        function updateParachuteDynamics(obj, M_aero_par, R_pe, dt)
            % Update parachute orientation
            %
            % Inputs:
            %   M_aero_par - Aerodynamic moment in parachute frame
            %   R_pe - Rotation matrix parachute to earth
            %   dt - Time step
            
            % Inertia
            I_total = [obj.I_p(1) + obj.a44; 
                       obj.I_p(2) + obj.a66; 
                       obj.I_p(3) + obj.a66];

            I_mat = diag(I_total);
            
            % Gravity in parachute frame
            g_earth = [0; 0; -obj.g];
            g_par = R_pe' * g_earth;
            
            % Gravity Moment
            M_grav_par = cross(obj.L_p, obj.m_c * g_par);
            
            % Total Moment
            M_total_par = M_aero_par + M_grav_par;
            
            % Rotation Equation
            gyro_term = cross(obj.omega_p, I_mat * obj.omega_p);
            domega_par = I_mat \ (M_total_par - gyro_term);
            
            % Integrate Angular Velocity
            obj.omega_p = obj.omega_p + domega_par * dt;
            
            % Update Quaternion
            dq = quatDerivative(obj.q_p, obj.omega_p);
            obj.q_p = obj.q_p + dq * dt;
            obj.q_p = obj.q_p / norm(obj.q_p);
        end
        
        function setOrientation(obj, q_initial, omega_initial)
            % Set initial parachute orientation
            obj.q_p = q_initial / norm(q_initial);
            obj.omega_p = omega_initial;
        end
        
        function [alpha, C_D, C_m] = getCurrentAeroState(obj, v_attach_earth)
            % Get current aerodynamic state
            R_pe = quatToRotMat(obj.q_p);
            v_rel_par = R_pe' * v_attach_earth;
            V_mag = norm(v_rel_par);
            
            if V_mag > 1e-6
                alpha = atan2(sqrt(v_rel_par(1)^2 + v_rel_par(2)^2), -v_rel_par(3));
            else
                alpha = 0;
            end
            
            C_D = obj.C_Dp_func(alpha);
            C_m = obj.C_mp_func(alpha);
        end
    end
end

%% Helper Functions
function R = quatToRotMat(q)
    % Convert quaternion to rotation matrix
    % Input: q = [q0; q1; q2; q3] 
    % q0 is scalaer term
    
    w = q(1); x = q(2); y = q(3); z = q(4);
    
    R = [1-2*(y^2+z^2),   2*(x*y-w*z),   2*(x*z+w*y);
         2*(x*y+w*z),   1-2*(x^2+z^2),   2*(y*z-w*x);
         2*(x*z-w*y),     2*(y*z+w*x), 1-2*(x^2+y^2)];
end

function dq = quatDerivative(q, omega)
    % Compute quaternion derivative
    % Inputs:
    %   q - Quaternion [q0; q1; q2; q3]
    %   omega - Angular velocity in body frame
    % Output:
    %   dq - Time derivative [dq0; dq1; dq2; dq3]
    
    q0 = q(1);
    qv = q(2:4);
    
    dq = 0.5 * [-qv' * omega;
                 q0 * omega + cross(qv, omega)];
end