
classdef Flight < handle&Exportable
    properties
        rocket_dyn
        controller
        environment
        navigation
        breakPoints

        flightParams
        totalTime
        currentTime
        x0
        dt
    end
    methods
        function obj = Flight(varargin)
            if varargin == 0
                return
            elseif varargin == 6
                rocket_dyn = varargin{1}; 
                controller = varargin{2}; 
                environment = varargin{3};
                navigation = varargin{4}; 
                breakPoints = varargin{5}; 
                flightParams = varargin{6};
                
                obj.rocket_dyn = rocket_dyn;
                obj.controller = controller;
                obj.environment = environment;
                obj.navigation = navigation;
                obj.breakPoints = breakPoints;
                obj.flightParams = flightParams;
    
                obj.dt = obj.flightParams.dt;
                obj.x0 = obj.flightParams.x0;
                obj.totalTime = obj.flightParams.totalTime;
                
                %obj.rocket_dyn.calc_moi()
            else
                % ERROR: Catches any other number of inputs
                error('Flight constructor expects either 0 (for loading) or 6 input arguments.');
            end
        end % wrap added by Zhanna S

        function restoreFromStruct(obj, s)
        % Restore all properties from a struct (bypasses SetAccess=private)
            f = fieldnames(s); % Get all field names from the struct 's'
            for i = 1:numel(f) % Loop thru each field
                try
                    obj.(f{i}) = s.(f{i}); % Assign struct to the object property
                catch ME % Warning if fails so it doesn't crash
                    warning('Could not restore %s: %s', f{i}, ME.message);
                end
            end
        end % function added by Zhanna S

        % TODO: Build setters for all attributes

        %function calc_moi()
            
        %end

        function [tArr, xArr, FbArr, MbArr] = runSim(obj)
            % This function propagates the dynamics using RK4
            
            % Need setters for this
            x = obj.x0(:);
            N = obj.totalTime / obj.dt;

            tArr = zeros(1,N);
            xArr = zeros(14, N);
            FbArr = zeros(3,N);
            MbArr = zeros(3,N);

            for ii = 2:N
                k1 = obj.rocket_model(x);
                k2 = obj.rocket_model(x + (k1)*obj.dt/2);
                k3 = obj.rocket_model(x + (k2)*obj.dt/2);
                k4 = obj.rocket_model(x + (k3)*obj.dt);
                
                obj.currentTime = ii * obj.dt;
                x = x + obj.dt*((1/6)*k1 + (1/3)*k2 + (1/3)*k3 + (1/6)*k4);

                tArr(ii) = obj.currentTime;
                xArr(:, ii) = x;

                % replace by Log Class
                [FbArr(:,ii), MbArr(:,ii)] = obj.get_forces_and_moments_wrapper(x);
            end
        end

        function x_dot = rocket_model(obj, state)
            h = state(4);
            %disturbances.wind = obj.environment.get_wind(h);  TODO
            disturbances.wind = 0;
            
            [F_b, M_b] = obj.rocket_dyn.get_forces_and_moments(state, disturbances, obj.currentTime);
            x_dot = obj.flatEarth_EOM(state, F_b, M_b);
        end
        
        % To avoid -> replace by Log class
        function [F_b, M_b] = get_forces_and_moments_wrapper(obj, state) %Duplicate of rocket_model method to get F_b and M_b
            h = state(4);
            %disturbances.wind = obj.environment.get_wind(h);  TODO
            disturbances.wind = 0;
            
            [F_b, M_b] = obj.rocket_dyn.get_forces_and_moments(state, disturbances, obj.currentTime);
        end

        function [state_dot] = flatEarth_EOM(state, F_b, M_b)   
            mass = state(1);
            %pos = state(2:4);  % not needed
            vel = state(5:7);
            quat = state(8:11);
            omega = state(12:14);

            mat_omega = [
                0, -omega(1), -omega(2), -omega(3);
                omega(1), 0, omega(3), -omega(2);
                omega(2), -omega(3), 0, omega(1);
                omega(3), omega(2), -omega(1), 0
            ];

            q = quat;
            C_B_I = Utils.dir_cosine(q);
            C_I_B = C_B_I.';

            %g = [0;0;-9.81];
			g = [0;0;0];

            F_I = (C_I_B * F_b) + mass * g;
            %Ib = obj.rocket_dyn.get_moi(mass);  % TODO
			Ib = 10 * eye(3);
            
            mass_dot = 0;
            pos_dot = vel;
            vel_dot = F_I / mass;
            quat_dot = 0.5 * mat_omega * quat;
            omega_dot = Ib\(M_b - cross(omega, Ib*omega));
            state_dot = [mass_dot; pos_dot; vel_dot; quat_dot; omega_dot];
        end

        function plotFlight(t_arr, x_arr)
            
            figure;
            plot3(x_arr(2, :), x_arr(3, :), x_arr(4, :), 'LineWidth', 2);
            grid on;
            xlabel('X Position (m)');
            ylabel('Y Position (m)');
            zlabel('Z Position (m)');
            title('3D Trajectory of the Flight');
            axis equal;
            view(3);
            
            % Plotting velocity components
            figure;
            hold on;
            plot(t_arr, x_arr(5, :), 'r', 'LineWidth', 2, 'DisplayName', 'V_x (m/s)');
            plot(t_arr, x_arr(6, :), 'g', 'LineWidth', 2, 'DisplayName', 'V_y (m/s)');
            plot(t_arr, x_arr(7, :), 'b', 'LineWidth', 2, 'DisplayName', 'V_z (m/s)');
            xlabel('Time (s)');
            ylabel('Velocity (m/s)');
            grid on;
            legend('show');
            title('Velocity Components');
       
            % Plotting angular velocity components
            figure;
            hold on;
            plot(t_arr, x_arr(12, :), 'r', 'LineWidth', 2, 'DisplayName', 'ω_x (rad/s)');
            plot(t_arr, x_arr(13, :), 'g', 'LineWidth', 2, 'DisplayName', 'ω_y (rad/s)');
            plot(t_arr, x_arr(14, :), 'b', 'LineWidth', 2, 'DisplayName', 'ω_z (rad/s)');
            xlabel('Time (s)');
            ylabel('Angular Velocity (rad/s)');
            grid on;
            legend('show');
            title('Angular Velocity Components');
            
            % Convert quaternion to Euler angles
            euler_angles = quat2eul(x_arr(8:11, :));

            % Plotting Euler angles
            figure;
            subplot(3, 1, 1);
            plot(t_arr, euler_angles(1, :), 'r', 'LineWidth', 2);
            ylabel('Roll (rad)');
            grid on;

            subplot(3, 1, 2);
            plot(t_arr, euler_angles(2, :), 'g', 'LineWidth', 2);
            ylabel('Pitch (rad)');
            grid on;

            subplot(3, 1, 3);
            plot(t_arr, euler_angles(3, :), 'b', 'LineWidth', 2);
            ylabel('Yaw (rad)');
            xlabel('Time (s)');
            grid on;
            sgtitle('Orientation (Euler Angles)');

        end
    end
end
