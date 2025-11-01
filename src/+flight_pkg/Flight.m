classdef Flight < handle
    properties
        rocketDynamics
        environment
        breakPoints

        flightParams
        totalTime
        currentTime
        currentXDot
        x0
        dt
    end
    methods
        function obj = Flight(rocketDynamics, environment, breakPoints, flightParams)
            obj.rocketDynamics = rocketDynamics;
            obj.environment = environment;
            obj.breakPoints = breakPoints;
            obj.flightParams = flightParams;

            obj.dt = obj.flightParams.dt;
            obj.x0 = obj.flightParams.x0;
            obj.totalTime = obj.flightParams.totalTime;
            
            %obj.rocket_dyn.calc_moi()
        end


        % TODO: Build setters for all attributes
            
        %end

        function [tArr, xArr, FbArr, MbArr, stateEstimateArr] = runSim(obj)
            % This function propagates the dynamics using RK4

            % Need setters for this
            x = obj.x0(:);
            N = obj.totalTime / obj.dt;
            obj.currentTime = 0.0;

            tArr = zeros(1,N);
            xArr = zeros(14, N);
            FbArr = zeros(3,N);
            MbArr = zeros(3,N);

            stateEstimateArr = zeros(14,N);

            for ii = 2:N
                ii
                obj.currentXDot = obj.rocketModel(x);

                disp('xdot')
                disp(obj.currentXDot)
                stateEstimateArr(:, ii) = obj.rocketDynamics.navigation.getStateEstimate(x, obj.currentXDot, obj.currentTime);
                
                disp('state estimate')
                disp(stateEstimateArr(:,ii))

                k1 = obj.rocketModel(x);
                k2 = obj.rocketModel(x + (k1)*obj.dt/2);
                k3 = obj.rocketModel(x + (k2)*obj.dt/2);
                k4 = obj.rocketModel(x + (k3)*obj.dt);

                obj.currentTime = ii * obj.dt;

                x = x + obj.dt*((1/6)*k1 + (1/3)*k2 + (1/3)*k3 + (1/6)*k4);

                % quaternion normalization
                quat = x(8:11);
                quatNorm = Utils.normalizeQuat(quat);
                x(8:11) = quatNorm;
    
                tArr(ii) = obj.currentTime;
                xArr(:, ii) = x;
                
                disp('x arr')
                disp(xArr(:, ii))

                % replace by Log Class
                [FbArr(:,ii), MbArr(:,ii)] = obj.get_forces_and_moments_wrapper(x);
            end
        end

        function x_dot = rocketModel(obj, state)
            altitude = state(4);
            %disturbances.wind = obj.environment.get_windspeed(altitude);
            disturbances.wind = zeros(3);
            
            [F_b, M_b] = obj.rocketDynamics.get_forces_and_moments(state, disturbances, obj.currentTime, obj.currentXDot, obj.breakPoints);
            x_dot = obj.flatEarth_EOM(state, F_b, M_b);
        end
        
        % To avoid -> replace by Log class
        function [F_b, M_b] = get_forces_and_moments_wrapper(obj, state) %Duplicate of rocket_model method to get F_b and M_b
            %disturbances.wind = obj.environment.get_wind(h);  TODO
            disturbances.wind = 0;
            
            [F_b, M_b] = obj.rocketDynamics.get_forces_and_moments(state, disturbances, obj.currentTime);
        end

        function [state_dot] = flatEarth_EOM(obj, state, F_b, M_b)   
            mass = state(1);
            vel = state(5:7);
            quat = state(8:11);
            omega = state(12:14);

            mat_omega = [
                0, -omega(1), -omega(2), -omega(3);
                omega(1), 0, omega(3), -omega(2);
                omega(2), -omega(3), 0, omega(1);
                omega(3), omega(2), -omega(1), 0
            ];


            % preventing mass to go below mass_dry
            mass = max(mass, obj.rocketDynamics.mass_dry); 

            [~, Ib] = obj.rocketDynamics.getCgMoi(mass);

            C_B_I = Utils.dir_cosine(quat);
            C_I_B = C_B_I.';

            g = [0;0;-9.81];

            F_b = F_b + C_B_I * g;
            
            mass_dot = obj.rocketDynamics.getMDot(obj.currentTime);
            pos_dot = C_I_B * vel;
            vel_dot = F_b / mass - cross(omega, vel);
            quat_dot = 0.5 * mat_omega * quat;
            omega_dot = Ib\(M_b - cross(omega, Ib*omega));
            state_dot = [mass_dot; pos_dot; vel_dot; quat_dot; omega_dot];
        end

        function plotFlight(obj, t_arr, x_arr)
            
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

            % Plot eurler angles

            
            % Convert quaternion to Euler angles
            euler_angles = rad2deg(quat2eul(x_arr(8:11, :).'));

            % Plotting Euler angles
            figure;
            subplot(3, 1, 1);
            plot(t_arr, euler_angles(:, 1), 'r', 'LineWidth', 2);
            ylabel('Roll (deg)');
            grid on;

            subplot(3, 1, 2);
            plot(t_arr, euler_angles(:, 2), 'g', 'LineWidth', 2);
            ylabel('Pitch (deg)');
            grid on;

            subplot(3, 1, 3);
            plot(t_arr, euler_angles(:, 3), 'b', 'LineWidth', 2);
            ylabel('Yaw (deg)');
            xlabel('Time (s)');
            grid on;
            sgtitle('Orientation (Euler Angles)');
        end
    end
end
