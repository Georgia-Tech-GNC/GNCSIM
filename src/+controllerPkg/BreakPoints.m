% Main Authors: Hridai Ambati (hambati3@gatech.edu)

classdef BreakPoints < handle

    properties
        rocketDynamics
        numBreakPoints
        maxSpeed
        
        nominalPos = [0, 0, 0].';
        %nominalVel = [up, yaw, pitch]
        nominalQuat = Utils.eulerToQuat([0, -90, 0]);
        nominalOmega = [0, 0, 0].';
        breakPointArr = [];
    end

    methods
        function obj = BreakPoints(rocketDynamics, numBreakPoints, maxSpeed)
            obj.rocketDynamics = rocketDynamics;
            obj.numBreakPoints = numBreakPoints;
            obj.maxSpeed = maxSpeed;
        end

        function x = xBar(obj, zq, params)
			alpha = params.alpha;
			speed = params.speed;

            mass = obj.rocketDynamics.massWet;
            pos = obj.nominalPos;
            vel = [speed*cosd(alpha), 0, speed*sind(alpha)].';
            quat = obj.nominalQuat;
            omega = [0, zq, 0].';
            x = [mass; pos; vel; quat; omega];
        end

        function u = uBar(obj, zu)
            u = [zu, 0, 0].';
        end
		
		function cost = objectiveFunc(obj, z, params)
			x = obj.xBar(z(1), params);
			u = obj.uBar(z(2));

			g = @(x) controllerPkg.pitchReduce(x);
			Jg = Utils.numericalJacobian(g, x);
			xDot = obj.rocketDynamics.rocketModel(x, u, 0, [], false);

            x3Dot = Jg * xDot;

			cost = norm(x3Dot(1:2));
		end

        function generateBreakPoints(obj)
            speedArr = linspace(10, obj.maxSpeed, obj.numBreakPoints);
            params.alpha = 0;
            obj.breakPointArr = controllerPkg.BreakPoint.empty(0, obj.numBreakPoints);
            for ii=1:length(speedArr)
                params.speed = speedArr(ii);
                breakPoint = obj.solveBreakPoint(params);
                obj.breakPointArr(ii) = breakPoint;
            end
        end

        function [A3, B3] = calculateROMpitch(obj, xStar, deflection)
			% Linearizes full dynamics and projects them onto 3-state Pitch space
			
			% Define function handles for numerical differentiation
            f = @(x, u) obj.rocketDynamics.rocketModel(x, u, 0, [], false, false);
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
            f = @(x, u) obj.rocketDynamics.rocketModel(x, u, 0, [], false, false);
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

        function breakPoint = solveBreakPoint(obj, params)
			z0 = [0, 0];
			%options = optimset('Display','iter','PlotFcns',@optimplotfval);

            objective_handle = @(z) obj.objectiveFunc(z, params);
            [zStar, ~] = fminsearch(@(z) objective_handle(z), z0);

			xStar = obj.xBar(zStar(1), params);
			uStar = obj.uBar(zStar(2));
            pitchStar = uStar(1);

            [A3, B3] = obj.calculateROMpitch(xStar, pitchStar);
            [A2, B2] = obj.calculateROMroll(xStar, pitchStar);

            breakPoint = controllerPkg.BreakPoint(params);

            % load breakpoint info into object
            breakPoint.zStar = zStar;
            breakPoint.xStar = xStar;
            breakPoint.uStar = uStar;
            breakPoint.A3 = A3;
            breakPoint.B3 = B3;
            breakPoint.A2 = A2;
            breakPoint.B2 = B2;   
            breakPoint.computeGain()
		end
            
            % TODO: Break trajectory based on accumulated velocity or
            % altitude and corresponding hyper parameters. Compute A and B
            % (x_dot = Ax + Bu)
            % for each break point
            
        
        
        % Depercated at the moment (Sorry Hridai)
        %{
        function compute_optimal_trajectory(obj, rocketDynamics)
            % TODO: Port Danil's work here 
        end

        function compute_dumb_trajectory(obj)

            % generate trivial trajectory
            x_0 = [0; 0; 0];
            x_f = [0; 0; 1000];     % 3x1
            dt = 0.1;               % s
            T = 20;             % s
            obj.nu
            
            % Grid and kinematics
            N  = max(2, floor(T/dt)+1);
            t  = (0:N-1)*dt;
            d  = x_f - x_0;                   % 3x1
            s  = linspace(0,1,N);             % 1xN (progress)
            pos = x_0 + d*s;                  % 3xN (implicit expansion)
            v   = d / ((N-1)*dt);             % 3x1 constant vel
            vel = repmat(v,1,N);              % 3xN
            
            % Outputs
            obj.x_optimal_time = [pos; vel];      % 6xN
            obj.u_optimal_time = zeros(obj.nu, obj.N_t);    % nu x N (all zeros)
            obj.time_optimal = t;
               
        end

        function generate_trajectory_av(obj)

            % Velocity and |Velocity|
            vel_optimal_time = obj.x_optimal_time(4:6,:);
            vel_optimal_time_mag = vecnorm(vel_optimal_time,2,1);
            
            % Integrate to find Max Accumulated Velocity
            dt = diff(time_optimal);

            raw_optimal_av = zeros(size(vel_optimal_time_mag));

            for i = 2:(size(vel_optimal_time,2))
                raw_optimal_av(i) = raw_optimal_av(i-1) + dt(i-1)*vel_optimal_time_mag(i-1);
            end

            max_av = raw_optimal_av(end);
            
            % Normalizing
            norm_optimal_av = raw_optimal_av./max_av;
            
            % New uniform grid
            step = 0.05;
            s_new = 0:step:1;
            s_new(end) = 1;
        end

        
        
        function compute_gains(obj, controller)
            % TODO: Call controller for each breakPoint and compute gains
            
        end

        function stability_analysis(obj)
            % TODO: Closed-loop stability analysis and classical controller
            % performance for each breakpoint (root locus, % overshoot,
            % settling time, rise time, peak time, etc)
        end

        function gain_scheduling_interpolated()
            
        end

        % Fill in properties as needed
        breakPointList;

        % Number of states and controls 
        nx;
        nu;

        % Trajectory with respect to time
        x_optimal_time;
        u_optimal_time;
        optimal_time;

        % Trajectory with respect to accumulated velocity
        x_optimal_av;
        u_optimal_av;
        optimal_av;

        N_av;
        %}

	end
end