% Main Authors: David Reynolds (dreynolds45@gatech.edu)

classdef ReduceLTI < handle
	% ReduceLTI  Class for generating and analyzing Reduced Order Models (ROM) 
    % for Linear Time-Invariant (LTI) flight dynamics.
    properties
        rocketDynamics		% Handle to the main flight/rocket physics model
    end

    methods
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

        function [A3, B3] = calculateROMpitch(obj, xStar, pitchControlInput, actuatorState, time)
			% Linearizes full dynamics and projects them onto 3-state Pitch space
			% Now uses actual actuatorState from the operating point

            disturbances.wind = [0, 0, 0].';

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
            B = B_aero * B_actuator; % (14x1)

            controllerPkg.Controller.check_controllability(A, B_aero);
            
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

        function [A2, B2] = calculateROMroll(obj, xStar, rollControlInput, actuatorState, time)
			% Linearizes full dynamics and projects them onto 2-state Roll space
			% Now uses actual actuatorState from the operating point

            disturbances.wind = [0, 0, 0].';

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
            B = B_aero * B_actuator; % (14x1)
            
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
            omega0 = log.states(13, 1);
            xStar(12:14) = [0; 0; 0];
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
	        % Maps full 14-state vector to Pitch states: [Alpha; q; Theta]
	        % https://ctms.engin.umich.edu/CTMS/index.php?example=AircraftPitch&section=SystemModeling
        
            arguments
                x14
                yaw = false;
	        end
        
	        if yaw
                %q = Utils.eulerToQuat([90 0 0]);
		        %DCM = Utils.dirCosine(q);
        
		        q0 = x14(8);
    	        q1 = x14(9);
    	        q2 = x14(10);
    	        q3 = x14(11);
        
		        quat = quaternion([q0', q1', q2', q3']);  % N x 4 matrix
                        quat_x90 = quaternion(Utils.eulerToQuat([-90, 0, 0])');
                        quat_rotated = quat * quat_x90;
        
		        DCM = Utils.dirCosine(compact(quat_x90));
		        x14(8:11) = compact(quat_rotated);
		        x14(5:7) = DCM * x14(5:7);
		        x14(12:14) = DCM * x14(12:14);
            end
        
            u = x14(5); v = x14(6); w = x14(7);
            q_body = x14(12:14);
	        
	        % angle of attack
	        alpha = atan2(w, u);
	        
            % pitch from quaternion
            qw = x14(8);
            qx = x14(9);
            qy = x14(10);
            qz = x14(11);
        
	        c = cos(pi/4);
	        s = sin(pi/4);
            
            % Manual Quaternion Multiplication: q_new = q * [c, 0, s, 0]
            qw_rot = c*qw - s*qy;
            qx_rot = c*qx - s*qz;
            qy_rot = c*qy + s*qw;
            qz_rot = c*qz + s*qx;
         
            % Extract Pitch (Theta) - Default
            % Note: Using asin avoids singularity issues better than atan2 for pitch
            theta = asin(2*(qw_rot*qy_rot - qx_rot*qz_rot));
        
        
	        % pitch rate (q) - Default
	        q = q_body(2); 
        
            x3 = [alpha; q; theta];
        end 

        function x2 = rollReduce(x14)
            % Maps full 14-state vector to Roll states: [Phi; p]
            
            % Roll rate (p) - Body frame rate is typically used directly
            p = x14(12); 
        
            qw = x14(8);
            qx = x14(9);
            qy = x14(10);
            qz = x14(11);
            
            %{
            c = cos(pi/4);
            s = sin(pi/4);
        
            % Manual Quaternion Multiplication:
            qw_rot = c*qw - s*qy;
            qx_rot = c*qx - s*qz;
            qy_rot = c*qy + s*qw;
            qz_rot = c*qz + s*qx;
            %}
        
            % Using standard ZYX roll formula on the new components
            phi = atan2(2*(qw*qx + qy*qz), 1 - 2*(qx^2 + qy^2));
            x2 = [p; phi];
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
