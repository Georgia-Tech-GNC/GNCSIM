% Main Authors: Hridai Ambati (hambati3@gatech.edu)

classdef BreakPoints < Handle

    properties
        rocketDynamics
        numBreakPoints
        maxSpeed
        
        nominalPos = [0, 0, 0].';
        %nominalVel = [up, yaw, pitch]
        nominalQuat = Utils.eulerToQuat([0, -90, 0]).';
        nominalOmega = [0, 0, 0].';

    end

    methods

        function obj = BreakPoints(rocketDynamics, numBreakPoints, maxSpeed)
            obj.rocketDynamics = rocketDynamics;
            obj.numBreakPoints = numBreakPoints;
            obj.maxSpeed = maxSpeed;
            %compute_dumb_trajectory();

        end

        function x = xBar(obj, zx, speed)
            mass = obj.rocketDynamics.rocketParams.massWet;
            pos = obj.nominalPos;
            vel = [speed*cos(zx), 0, speed*sin(zx)].';
            quat = obj.nominalQuat;
            omega = obj.nominalOmega;
            x = [mass; pos; vel; quat; omega];
        end

        function u = uBar(obj, zu)
            u = [zu, 0, 0].';
        end



        function generateBreakPoints(obj, rocketDynamics)
            
            
            
            % TODO: Break trajectory based on accumulated velocity or
            % altitude and corresponding hyper parameters. Compute A and B
            % (x_dot = Ax + Bu)
            % for each break point
            
        end
        
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