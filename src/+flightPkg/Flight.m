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

        function [tArr, xArr, fbArr, mbArr, stateEstimateArr] = runSim(obj)
            % This function propagates the dynamics using RK4

            % Need setters for this
            x = obj.x0(:);
            N = obj.totalTime / obj.dt;
            obj.currentTime = 0.0;

            tArr = zeros(1,N);
            xArr = zeros(14, N);
            fbArr = zeros(3,N);
            mbArr = zeros(3,N);

            stateEstimateArr = zeros(14,N);

            for ii = 2:N
                
                obj.currentXDot = obj.rocketModel(x);

                %disp('xdot')
                %disp(obj.currentXDot)
                stateEstimateArr(:, ii) = obj.rocketDynamics.navigation.getStateEstimate(x, obj.currentXDot, obj.currentTime);

                %disp('state estimate')
                %disp(stateEstimateArr(:,ii))

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

                %disp('x arr')
                %disp(xArr(:, ii))

                % replace by Log Class
                [fbArr(:,ii), mbArr(:,ii)] = obj.getForcesMomentsWrapper(x);
            end
        end

        function xDot = rocketModel(obj, state)
            altitude = state(4);
            %disturbances.wind = obj.environment.getWindspeed(altitude);
            disturbances.wind = zeros(3);

            [fB, mB] = obj.rocketDynamics.getForcesMoments(state, disturbances, obj.currentTime, obj.currentXDot, obj.breakPoints);
            xDot = obj.flatEarth_EOM(state, fB, mB);
        end
        
        % To avoid -> replace by Log class
        function [fB, mB] = getForcesMomentsWrapper(obj, state) %Duplicate of rocket_model method to get fB and mB
            %disturbances.wind = obj.environment.getWind(h);  TODO
            disturbances.wind = 0;

            [fB, mB] = obj.rocketDynamics.getForcesMoments(state, disturbances, obj.currentTime);
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
