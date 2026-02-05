% Main Authors: David Reynolds (dreynolds45@gatech.edu)

classdef ReduceLTI < handle
	% ReduceLTI  Class for generating and analyzing Reduced Order Models (ROM) 
    % for Linear Time-Invariant (LTI) flight dynamics.
    properties
        rocketDynamics		% Handle to the main flight/rocket physics model
    end

    methods
        % Constructor
        function obj = ReduceLTI(rocketDynamics)
            obj.rocketDynamics = rocketDynamics;
		end
		
        function [sysPitch, sysRoll] = generateROM(obj, xStar)
			% Generates LTI State-Space (ss) objects for Pitch and Roll channels

			% Pitch Dynamics
            [A3, B3] = obj.calculateROMpitch(xStar(:), 0);
            C = [0 0 1];  % output is pitch (theta)
            D = 0;
            sysPitch = ss(A3, B3, C, D);
			
			% Roll Dynamics
            [A2, B2] = obj.calculateROMroll(xStar(:), 0);
            C = [0 1];  % output is roll (phi)
            D = 0;
            sysRoll = ss(A2, B2, C, D);
		end

        function [A3, B3] = calculateROMpitch(obj, xStar, pitchControlInput, actuatorState, time, useActuatorModel)
			% Linearizes full dynamics and projects them onto 3-state Pitch space
			% Now uses actual actuatorState from the operating point

            disturbances.wind = [0, 0, 0].';
            
            if useActuatorModel
			    % Define function handles for numerical differentiation
                f = @(x, deflection) obj.rocketDynamics.rocketModel(x, deflection, disturbances, time, [], true, true);
                h = @(u, deflection) obj.rocketDynamics.actuatorModel.actuatorDynamics(u, deflection);
                fx = @(x) f(x, actuatorState);
                fdel = @(del_in) f(xStar, del_in);
                hu = @(u_in) h([0, u_in, 0].', actuatorState);
    
			    % Calculate Full Jacobians
                A = Utils.numericalJacobian(fx, xStar); % (14x14)
                B_aero = Utils.numericalJacobian(fdel, actuatorState); % (14x4)
                B_actuator = Utils.numericalJacobian(hu, pitchControlInput); % (4x1)
                % B_actuator = obj.rocketDynamics.actuatorModel.allocationMatrix(:, 2); % no actuator lag
                B = B_aero * B_actuator; % (14x1)
            else
                % Define function handles for numerical differentiation
                f = @(x, u) obj.rocketDynamics.rocketModelControlWrapper(x, u, disturbances, time, [], false, false);
                fx = @(x) f(x, [0, pitchControlInput, 0].');
                fu = @(u_in) f(xStar, [0, u_in, 0].');
    
			    % Calculate Full Jacobians
                A = Utils.numericalJacobian(fx, xStar); % (14x14)
                B = Utils.numericalJacobian(fu, pitchControlInput); % (14x1)
            end
            
			% Project to Reduced Space (3x3)
            g = @(x) controllerPkg.ReduceLTI.pitchReduce(x);
			
			% Reduction mapping matrix (3x14)
            Jg = Utils.numericalJacobian(g, xStar);

			% Pseudo-inverse for projection (14x3)
            T = pinv(Jg);
			
			% Projected System Matrix (3x3)
            A3 = Jg*A*T;

			% Projected Control Matrix (3x1)
            B3 = Jg*B;
        end

        function [A2, B2] = calculateROMroll(obj, xStar, rollControlInput, actuatorState, time, useActuatorModel)
			% Linearizes full dynamics and projects them onto 2-state Roll space
			% Now uses actual actuatorState from the operating point

            disturbances.wind = [0, 0, 0].';
            if useActuatorModel
			    % Define function handles for numerical differentiation
                f = @(x, deflection) obj.rocketDynamics.rocketModel(x, deflection, disturbances, time, [], true, true);
                h = @(u, deflection) obj.rocketDynamics.actuatorModel.actuatorDynamics(u, deflection);
                fx = @(x) f(x, actuatorState);
                fdel = @(del_in) f(xStar, del_in);
                hu = @(u_in) h([u_in, 0, 0].', actuatorState);
    
			    % Calculate Full Jacobians
                A = Utils.numericalJacobian(fx, xStar); % (14x14)
                B_aero = Utils.numericalJacobian(fdel, actuatorState); % (14x4)
                B_actuator = Utils.numericalJacobian(hu, rollControlInput); % (4x1)
                % B_actuator = obj.rocketDynamics.actuatorModel.allocationMatrix(:, 1);
                B = B_aero * B_actuator; % (14x1)
            else
                % Define function handles for numerical differentiation
                f = @(x, u) obj.rocketDynamics.rocketModelControlWrapper(x, u, disturbances, time, [], true, true);
                fx = @(x) f(x, [rollControlInput, 0, 0].');
                fu = @(u_in) f(xStar, [u_in, 0, 0].');
    
			    % Calculate Full Jacobians
                A = Utils.numericalJacobian(fx, xStar); % (14x14)
                B = Utils.numericalJacobian(fu, rollControlInput); % (14x1)
            end
            
			% Project to Reduced Space (2x2)
            g = @(x) controllerPkg.ReduceLTI.rollReduce(x);
			
			% Reduction mapping matrix (2x14)
            Jg = Utils.numericalJacobian(g, xStar);

			% Pseudo-inverse for projection (14x2)
            T = pinv(Jg);
			
			% Projected System Matrix (2x2)
            A2 = Jg*A*T;

			% Projected Control Matrix (2x1)
            B2 = Jg*B;
        end

        function performComparison(obj, log)
            % Compares the Linearized ROM against the full Nonlinear Simulation Log
            % NOTE: Make sure thrust and gravity is off for fair comparison

            % Get LTI System
            xStar = log.states(:, 1);
            omega0 = log.states(StateIndices.omega_y, 1);
            xStar(StateIndices.omega) = [0; 0; 0];
            [sys, ~] = obj.generateROM(xStar);

            time = log.time;
            x0 = [0; omega0; 0];
            [~, ~, X] = initial(sys, x0, time);
            
            figure;
            subplot(2, 1, 1);
            
			% Compare Angular Velocity
            wyNonLTI = log.getState('wy');
            plot(time, rad2deg(wyNonLTI), 'g', 'LineWidth', 2, 'DisplayName', 'Nonlinear');
            hold on
            plot(time, rad2deg(X(:, 2)), 'b', 'LineWidth', 2, 'DisplayName', 'Linear')
            xlabel('\textbf{Time (s)}', 'Interpreter', 'latex');
            ylabel('\textbf{Angular Velocity (deg/s)}', 'Interpreter', 'latex');
            grid on;
            legend;
			
			% Compare Euler Angles
            subplot(2, 1, 2);
            
            % Euler Comparison
            q0 = log.getState('q0');
            q1 = log.getState('q1');
            q2 = log.getState('q2');
            q3 = log.getState('q3');
			
			% Adjust frame (90 deg Y rotation)
            quat = quaternion([q0', q1', q2', q3']);  % N x 4 matrix
            quat_y90 = quaternion(Utils.eulerToQuat([0, 90, 0])');
            quat_rotated = quat * quat_y90;

            eul = rad2deg(quat2eul(quat_rotated, 'ZYX'));
            
            plot(time, eul(:, 2), 'g', 'LineWidth', 2, 'DisplayName', 'Nonlinear');
            hold on
            plot(time, rad2deg(X(:, 3)), 'b', 'LineWidth', 2, 'DisplayName', 'Linear')
            ylabel('\textbf{Euler Angles (deg)}', 'Interpreter', 'latex')
            xlabel('\textbf{Time (s)}', 'Interpreter', 'latex')
            legend;
            grid on;
		end
    end

    methods (Static)
		function x3 = pitchReduce(x14, yaw)
    		% Maps full 14-state vector to Pitch (or Yaw) states: [Alpha/Beta; q/r; Theta/Psi]
    		% Reference: https://ctms.engin.umich.edu/CTMS/index.php?example=AircraftPitch&section=SystemModeling
		
    		arguments
        		x14
        		yaw = false;
    		end
		
    		% Get velocities
    		u = x14(StateIndices.vx); 
    		v = x14(StateIndices.vy); 
    		w = x14(StateIndices.vz);
    		omega = x14(StateIndices.omega);
		
    		% Target orientation: Vertical Up [0, -90, 0]
    		q0 = Utils.eulerToQuat([0, -90, 0]);
		
    		if yaw
        		% --- YAW REDUCTION LOGIC ---
        		% Alpha becomes Beta (sideslip)
        		angle_of_attack = atan2(v, u); 
        		% Rate becomes r (yaw rate)
        		rate = -omega(3); 
        		
        		% Rotate target by 90 deg to extract Yaw error into the 'theta' slot
        		% This effectively maps the Yaw deviation to the state output
        		[psi_err, ~, ~] = controllerPkg.ReduceLTI.getEulerErrors(x14, q0);
        		attitude_err = -psi_err;
    		else
        		% --- PITCH REDUCTION LOGIC ---
        		% Standard Alpha
        		angle_of_attack = atan2(w, u);
        		% Rate is q (pitch rate)
        		rate = omega(2);
        		
        		% Extract Pitch error relative to vertical
        		[~, theta_err, ~] = controllerPkg.ReduceLTI.getEulerErrors(x14, q0);
        		attitude_err = theta_err;
    		end
		
    		x3 = [angle_of_attack; rate; attitude_err];
		end
		
		function x2 = rollReduce(x14)
    		% Maps full 14-state vector to Roll states: [p; Phi]
    		p = x14(StateIndices.omega_x);
    		
    		q0 = Utils.eulerToQuat([0, -90, 0]);
    		[~, ~, phi_err] = controllerPkg.ReduceLTI.getEulerErrors(x14, q0);
    		
    		x2 = [p; phi_err];
		end
		
		%% Helper to calculate clean errors relative to any target (e.g., vertical)
		function [psi, theta, phi] = getEulerErrors(x14, q_target)
    		q_actual = x14(StateIndices.quat); 
    		
    		% Error Quaternion: q_err = q_target^-1 * q_actual
    		q_inv = [q_target(1); -q_target(2:4)];
    		
    		w1 = q_inv(1); x1 = q_inv(2); y1 = q_inv(3); z1 = q_inv(4);
    		w2 = q_actual(1); x2 = q_actual(2); y2 = q_actual(3); z2 = q_actual(4);
    		
    		qe_w = w1*w2 - x1*x2 - y1*y2 - z1*z2;
    		qe_x = w1*x2 + x1*w2 + y1*z2 - z1*y2;
    		qe_y = w1*y2 - x1*z2 + y1*w2 + z1*x2;
    		qe_z = w1*z2 + x1*y2 - y1*x2 + z1*w2;
    		
    		% Standard ZYX extraction from the error quaternion
    		% Since error is small, this is stable and singularity-free
    		phi   = real(atan2(2*(qe_w*qe_x + qe_y*qe_z), 1 - 2*(qe_x^2 + qe_y^2)));
    		theta = real(asin(2*(qe_w*qe_y - qe_z*qe_x)));
    		psi   = real(atan2(2*(qe_w*qe_z + qe_x*qe_y), 1 - 2*(qe_y^2 + qe_z^2)));
		end

		function drawROM(sys)
            % Simulates the LTI system response to an initial condition and plots
            x0 = [0; 0.1; 0]; % Initial perturbation
            tspan = 0:0.01:10; 
            [~, t, X] = initial(sys, x0, tspan);
            
            figure;
            X = rad2deg(X);
            
            % Plot Alpha (Angle of Attack)
            subplot(3, 1, 1);
            plot(t, X(:, 1), 'r', 'LineWidth', 1.5);
            ylabel('Angle of Attack \alpha (deg)');
            title('Initial Condition Response of System States');
            grid on;
            
            % Plot Pitch Rate (q)
            subplot(3, 1, 2);
            plot(t, X(:, 2), 'g', 'LineWidth', 1.5);
            ylabel('Pitch Rate (deg/s');
            grid on;
            
            % Plot Pitch Angle (Theta)
            subplot(3, 1, 3);
            plot(t, X(:, 3), 'b', 'LineWidth', 1.5);
            xlabel('Time (s)');
            ylabel('Pitch (deg)');
            grid on;
        end
    end
end
