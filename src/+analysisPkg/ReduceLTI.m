% Main Authors: David Reynolds (dreynolds45@gatech.edu)

classdef ReduceLTI < handle
	% ReduceLTI  Class for generating and analyzing Reduced Order Models (ROM) 
    % for Linear Time-Invariant (LTI) flight dynamics.
    properties
        flight		% Handle to the main flight/rocket physics model
    end

    methods
        function obj = ReduceLTI(flight)
            obj.flight = flight;
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

        function [A3, B3] = calculateROMpitch(obj, xStar, deflection)
			% Linearizes full dynamics and projects them onto 3-state Pitch space
			
			% Define function handles for numerical differentiation
            f = @(x, u) obj.flight.rocketDynamics.rocketModel(x, u, 0, [], false, false);
            fx = @(x) f(x, [deflection, 0, 0].');
            fu = @(u_in) f(xStar, [u_in, 0, 0].');

			% Calculate Full Jacobians (14x14)
            A = Utils.numericalJacobian(fx, xStar);
            B = Utils.numericalJacobian(fu, deflection);
            
			% Project to Reduced Space (3x3)
            g = @(x) controllerPkg.pitchReduce(x);
			
			% Reduction mapping matrix (3x14)
            Jg = Utils.numericalJacobian(g, xStar);

			% Pseudo-inverse for projection (14x3)
            T = pinv(Jg);
			
			% Projected System Matrix
            A3 = Jg*A*T;

			% Projected Control Matrix
            B3 = Jg*B;
        end

        function [A2, B2] = calculateROMroll(obj, xStar, deflection)
			% Linearizes full dynamics and projects them onto 2-state Roll space
			
			% Define function handles for numerical differentiation
            f = @(x, u) obj.flight.rocketDynamics.rocketModel(x, u, 0, [], false, false);
            fx = @(x) f(x, [0, 0, deflection].');
            fu = @(u_in) f(xStar, [0, 0, u_in].');

			% Calculate Full Jacobians (14x14)
            A = Utils.numericalJacobian(fx, xStar);
            B = Utils.numericalJacobian(fu, deflection);
            
			% Project to Reduced Space (2x2)
            g = @(x) controllerPkg.rollReduce(x);
			
			% Reduction mapping matrix (2x14)
            Jg = Utils.numericalJacobian(g, xStar);

			% Pseudo-inverse for projection (14x3)
            T = pinv(Jg);
			
			% Projected System Matrix
            A2 = Jg*A*T;

			% Projected Control Matrix
            B2 = Jg*B;
		end

		function drawROM(obj, sys)
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
end
