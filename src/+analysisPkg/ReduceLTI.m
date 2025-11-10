
classdef ReduceLTI < handle
    properties
        flight
    end

    methods
        function obj = ReduceLTI(flight)
            obj.flight = flight;
        end

        function calculateROM(obj, xStar, uStar)
            f = @(x, u) obj.flight.rocketModelDeflection(x, u);
            fx = @(x) f(x, uStar);
            fu = @(u) f(xStar, u);
            A = Utils.numericalJacobian(fx, xStar);
            B = Utils.numericalJacobian(fu, uStar);
            
            g = @(x) obj.reduce(x);

            Jg = Utils.numericalJacobian(g, xStar);
            T = pinv(Jg);

            A3 = Jg*A*T;

            B = []; % No input term (u=0)
            C = [0 0 1];
            D = [0];
            
            sys = ss(A3, B, C, D);

            tspan = 0:0.001:1; % Specify the time vector
            x0 = [0; 0.5; 0];      % Initial conditions
            
            [~, t, X] = initial(sys, x0, tspan);

            figure;

            % Subplot 1: State x_1(t)
            subplot(3, 1, 1);
            plot(t, X(:, 1), 'r', 'LineWidth', 1.5);
            ylabel('State x_1');
            title('Initial Condition Response of System States');
            grid on;
            
            % Subplot 2: State x_2(t)
            subplot(3, 1, 2);
            plot(t, X(:, 2), 'g', 'LineWidth', 1.5);
            ylabel('State x_2');
            grid on;
            
            % Subplot 3: State x_3(t)
            subplot(3, 1, 3);
            plot(t, X(:, 3), 'b', 'LineWidth', 1.5);
            xlabel('Time (s)');
            ylabel('State x_3');
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
