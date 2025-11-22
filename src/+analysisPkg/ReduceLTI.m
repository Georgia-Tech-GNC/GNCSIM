
classdef ReduceLTI < handle
    properties
        flight
    end

    methods
        function obj = ReduceLTI(flight)
            obj.flight = flight;
        end

        function sys = generateROM(obj, xStar)
            uStar = [0 0 0].';
            [A, B] = obj.calculateROM(xStar(:), uStar);
            C = [0 0 1];
            D = 0;
            
            sys = ss(A, B, C, D);
        end

        function [A3, B3] = calculateROM(obj, xStar, uStar)
            f = @(x, u) obj.flight.rocketModelDeflection(x, u);
            fx = @(x) f(x, uStar);
            fu = @(u) f(xStar, u);
            A = Utils.numericalJacobian(fx, xStar);
            B = Utils.numericalJacobian(fu, uStar);
            
            g = @(x) obj.reduce(x);

            Jg = Utils.numericalJacobian(g, xStar);
            T = pinv(Jg);

            A3 = Jg*A*T;
            B3 = Jg*B;
        end

        function drawROM(obj, sys)
            x0 = [0; 0.1; 0];
            tspan = 0:0.01:10; 
            [~, t, X] = initial(sys, x0, tspan);

            figure;
            X = rad2deg(X);

            % Subplot 1: State x_1(t)
            subplot(3, 1, 1);
            plot(t, X(:, 1), 'r', 'LineWidth', 1.5);
            ylabel('Angle of Attack \alpha (deg)');
            title('Initial Condition Response of System States');
            grid on;

            % Subplot 2: State x_2(t)
            subplot(3, 1, 2);
            plot(t, X(:, 2), 'g', 'LineWidth', 1.5);
            ylabel('Pitch Rate (deg/s');
            grid on;

            % Subplot 3: State x_3(t)
            subplot(3, 1, 3);
            plot(t, X(:, 3), 'b', 'LineWidth', 1.5);
            xlabel('Time (s)');
            ylabel('Pitch (deg)');
            grid on;
        end

        function performComparison(obj, log)
            % NOTE: Make sure thrust and gravity is off

            % Get LTI System
            xStar = log.states(:, 1);
            omega0 = log.states(13, 1);
            xStar(12:14) = [0; 0; 0];
            sys = obj.generateROM(xStar);

            time = log.time;
            x0 = [0; omega0; 0];
            [~, ~, X] = initial(sys, x0, time);
            
            figure;
            subplot(2, 1, 1);
            

            wyNonLTI = log.getState('wy');
            plot(time, rad2deg(wyNonLTI), 'g', 'LineWidth', 2, 'DisplayName', 'Nonlinear');
            hold on
            plot(time, rad2deg(X(:, 2)), 'b', 'LineWidth', 2, 'DisplayName', 'Linear')
            xlabel('\textbf{Time (s)}', 'Interpreter', 'latex');
            ylabel('\textbf{Angular Velocity (deg/s)}', 'Interpreter', 'latex');
            grid on;
            legend;

            subplot(2, 1, 2);
            
            % Euler Comparison
            q0 = log.getState('q0');
            q1 = log.getState('q1');
            q2 = log.getState('q2');
            q3 = log.getState('q3');

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

        function x3 = reduce(obj, x14)
            u = x14(5);
            v = x14(6);
            w = x14(7);
            q_body = x14(12:14); % p,q,r
        
            % angle of attack
            alpha = atan2(w, u);
        
            % pitch from quaternion
            qw = x14(8);
            qx = x14(9);
            qy = x14(10);
            qz = x14(11);
            theta = atan2(2*(qw*qy + qx*qz), 1 - 2*(qy^2 + qz^2));
        
            % pitch rate
            q = q_body(2); % body y-axis rate
        
            x3 = [alpha; q; theta];
        end
    end
end
