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

        function log = runSim(obj, log)
            % This function propagates the dynamics using RK4

            % Need setters for this
            obj.currentTime = 0.0;
            x = obj.x0(:);
            N = obj.totalTime / obj.dt;

            log.initializeTimestep(obj.currentTime);
            log.add(obj.currentTime, 'states', x);

            obj.currentXDot = obj.rocketModel(x, log);

            for ii = 1:N
                k1 = obj.currentXDot;
                k2 = obj.rocketModel(x + (k1)*obj.dt/2);
                k3 = obj.rocketModel(x + (k2)*obj.dt/2);
                k4 = obj.rocketModel(x + (k3)*obj.dt);

                obj.currentTime = ii * obj.dt;

                x = x + obj.dt*((1/6)*k1 + (1/3)*k2 + (1/3)*k3 + (1/6)*k4);

                quat = x(8:11);
                quatNorm = Utils.normalizeQuat(quat);
                x(8:11) = quatNorm;     

                log.initializeTimestep(obj.currentTime);
                log.add(obj.currentTime, 'states', x);

                obj.currentXDot = obj.rocketModel(x, log);
            end
        end

        function xDot = rocketModel(obj, state, log)
            arguments
                obj
                state 
                log = []  % Optional, defaults to empty
            end

            altitude = state(4);
            %disturbances.wind = obj.environment.getWindspeed(altitude);
            disturbances.wind = zeros(3);

            [fB, mB] = obj.rocketDynamics.getForcesMoments(state, disturbances, obj.currentTime, obj.currentXDot, obj.breakPoints, log);
            xDot = obj.flatEarth_EOM(state, fB, mB);

            if ~isempty(log)
                log.add(obj.currentTime, 'statedots', xDot);
            end
        end

        function xDot = rocketModelDeflection(obj, state, control)
            % Control: [Mx, My, Mz]
            altitude = state(4);
            %disturbances.wind = obj.environment.getWindspeed(altitude);
            disturbances.wind = zeros(3);

            [fB, mB] = obj.rocketDynamics.getForcesMoments(state, disturbances, obj.currentTime, obj.currentXDot, obj.breakPoints);
            
            xDot = obj.flatEarth_EOM(state, fB, mB + control(:));
        end
        
        function [stateDot] = flatEarth_EOM(obj, state, fB, mB)
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


            % preventing mass to go below massDry
            mass = max(mass, obj.rocketDynamics.massDry);

            [~, Ib] = obj.rocketDynamics.getCgMoi(mass);

            cBI = Utils.dirCosine(quat);
            cIB = cBI.';

            g = [0;0;-9.81];
            fG = mass * g;

            fB = fB + cBI * fG;

            massDot = obj.rocketDynamics.getMDot(obj.currentTime);
            posDot = cIB * vel;
            velDot = fB / mass - cross(omega, vel);
            quatDot = 0.5 * mat_omega * quat;
            omegaDot = Ib\(mB - cross(omega, Ib*omega));
            stateDot = [massDot; posDot; velDot; quatDot; omegaDot];
        end

        function plotFlight(obj, t_arr, x_arr)
            t_arr = t_arr(2:end);
            x_arr = x_arr(:, 2:end);

            figure;
            plot3(x_arr(2, :), x_arr(3, :), x_arr(4, :), 'LineWidth', 2);
            grid on;
            xlabel('X Position (m)');
            ylabel('Y Position (m)');
            zlabel('Z Position (m)');
            title('3D Trajectory of the Flight');
            axis equal;
            view(3);

            figure;
            plot(t_arr, x_arr(1, :));
            grid on;
            xlabel('Time (s)');
            ylabel('Mass (kg)');
            legend('show');
            title('Mass');
            
            % Plotting velocity components
            figure;
            hold on;
            plot(t_arr, x_arr(5, :), 'r', 'LineWidth', 2, 'DisplayName', 'V_x (m/s)');
            plot(t_arr, x_arr(6, :), 'g', 'LineWidth', 2, 'DisplayName', 'V_y (m/s)');
            plot(t_arr, x_arr(7, :), 'b', 'LineWidth', 2, 'DisplayName', 'V_z (m/s)');
            xlabel('Time (s)');
            ylabel('Velocity (m/s)');
            grid on;
            hold off;
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
            hold off;
            legend('show');
            title('Angular Velocity Components');

            % Plot eurler angles
            N = size(x_arr(8:11, :), 2);
            vertical_quat_arr = zeros(N, 4);
            euler_angles = zeros(N, 3);

            for ii=1:N
                quat = quaternion(x_arr(8:11, ii).');
                quat_y90 = quaternion(Utils.eulerToQuat([0 -90 0]).');
                vertical_quat_arr = quat * quat_y90;

                % Convert quaternion to Euler angles
                euler_angles(ii, :) = rad2deg(quat2eul(vertical_quat_arr));
            end

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
