classdef TrajectoryOptimization
    % TrajectoryOptimization: class to form a nominal trajectory for a
    % rocket flight. Currently utilizes direct collocation through Matthew
    % Kelly's OptimTraj library
    
    properties
        idx StateIndices = StateIndices;
        turnQuaternion (4,1) double
        turnTime (1,1) double
        maxAoA double = [] % radians
        noControlTime double = 3;
    end
    
    methods
        function obj = TrajectoryOptimization(turnQuaternion, turnTime, maxAoA)
            arguments
                turnQuaternion (4,1) double = zeros(4,1) % Can pass in nothing
                turnTime       double = 0
                maxAoA         double = []
            end
            obj.turnQuaternion = turnQuaternion;
            obj.turnTime = turnTime;
            obj.maxAoA = deg2rad(maxAoA);
        end

        function Jp = pathObjectiveFunction(obj, t, x, u)
            arguments
                obj
                t (1,:) double
                x (:,:) double
                u (:,:) double
            end

            [~,Nt] = size(x);
            Jp = zeros(1, Nt);
            for i = 1:Nt
                Jp(i) = sum(x(obj.idx.omega, i).^2) + sum(u(:,i).^2);
            end

        end

        function Jb = boundaryObjectiveFunction(obj, t0, x0, tF, xF)
            arguments
                obj
                t0 (1,1) double
                x0 (:,1) double
                tF (1,1) double
                xF (:,1) double
            end
            
            [~, Nt] = size(x0);
            Jb = zeros(1,Nt);

        end

        function [Cineq, Ceq] = pathConstraintFunction(obj, t, x, u)
            arguments
                obj
                t (1,:) double
                x (:,:) double
                u (:,:) double
            end
        
            Nt = numel(t);
            q  = x(obj.idx.quat, :);
        
            % Turn
            kTurn = [];
            if obj.turnTime > t(1) && obj.turnTime < t(end)
                [~, kTurn] = min(abs(t - obj.turnTime));
            end
        
            % Quaternion norm
            kNorm = setdiff(2:Nt, kTurn);
            Ceq = (vecnorm(q(:, kNorm)) - 1).';
        
            % Turn attitude
            if ~isempty(kTurn)
                Ceq = [Ceq; q(:, kTurn) - obj.turnQuaternion];
            end
        
            % Early ascent
            hold = find(t <= obj.noControlTime);
            holdFree = hold(hold > 1);
            lat = [obj.idx.vel(2:3), obj.idx.omega, obj.idx.pos(1:2)];
            Ceq = [Ceq; reshape(u(:, hold), [], 1); reshape(x(lat, holdFree), [], 1)];
        
            % AoA limit
            Cineq = [];
            if ~isempty(obj.maxAoA)
                v = x(obj.idx.vel, :);
                Cineq = (v(2,:).^2 + v(3,:).^2 - (tan(obj.maxAoA) * v(1,:)).^2).';
            end
        end

        function [Cineq, Ceq]= boundaryConstraintFunction(obj, t0, x0, tF, xF)
            arguments
                obj
                t0 (1,1) double
                x0 (:,1) double
                tF (1,1) double
                xF (:,1) double
            end
            Cineq = [];

            Ceq = [xF(obj.idx.vel(2:3)); % zero AoA
                   xF(obj.idx.omega)]; % zero angular rates

        end

        function []= plotNominalTrajectory(obj, nominalTrajectory)
            arguments
                obj
                nominalTrajectory struct
            end

            t = nominalTrajectory.time;

            pos_east = nominalTrajectory.states(obj.idx.pos(1), :);
            pos_north = nominalTrajectory.states(obj.idx.pos(2), :);
            pos_up = nominalTrajectory.states(obj.idx.pos(3), :);
            body_u = nominalTrajectory.states(obj.idx.vel(1), :);
            body_v = nominalTrajectory.states(obj.idx.vel(2), :);
            body_w = nominalTrajectory.states(obj.idx.vel(3), :);
            q0 = nominalTrajectory.states(obj.idx.quat(1), :);
            q1 = nominalTrajectory.states(obj.idx.quat(2), :);
            q2 = nominalTrajectory.states(obj.idx.quat(3), :);
            q3 = nominalTrajectory.states(obj.idx.quat(4), :);
            p = nominalTrajectory.states(obj.idx.omega(1), :);
            q = nominalTrajectory.states(obj.idx.omega(2), :);
            r = nominalTrajectory.states(obj.idx.omega(3), :);

            rollCmd = nominalTrajectory.controlInputs(1, :);
            pitchCmd = nominalTrajectory.controlInputs(2, :);
            yawCmd = nominalTrajectory.controlInputs(3, :);

            actuator1 = nominalTrajectory.actuatorState(1, :);
            actuator2 = nominalTrajectory.actuatorState(2, :);

            figure(1); clf % Position in ENU
            plot3(pos_east, pos_north, pos_up)
            xlabel('North', 'FontSize', 15)
            ylabel('East', 'FontSize', 15)
            zlabel('Down', 'FontSize', 15)
            axis([min(pos_east) max(pos_east) min(pos_north) max(pos_north) min(pos_up) max(pos_up)])
            title('3D Trajectory', 'FontSize', 20)

            figure(2); clf % Body velocity
            plot(t, body_u)
            hold on
            plot(t, body_v)
            plot(t, body_w)
            xlabel('Time (s)', 'FontSize', 15)
            ylabel('Velocity (m/s)', 'FontSize', 15)
            legend('u', 'v', 'w', 'FontSize', 18, 'Location', 'best')
            title('Body Velocity Components', 'FontSize', 20)


            figure(3); clf % Quaternion plot
            plot(t,q0)
            hold on
            plot(t,q1)
            plot(t,q2)
            plot(t,q3)
            xlabel('Time(s)', 'FontSize', 15)
            ylabel('Quaternion Component Magnitude', 'FontSize', 15)
            legend('q0', 'q1', 'q2', 'q3', 'FontSize', 18, 'Location', 'best')
            title('Quaternion Components', 'FontSize', 20)

            figure(4); clf % Angular rates
            plot(t,p)
            hold on
            plot(t,q)
            plot(t,r)
            xlabel('Time(s)', 'FontSize', 15)
            ylabel('Angular Velocity (rad/s)', 'FontSize', 15)
            legend('p', 'q', 'r', 'FontSize', 18, 'Location', 'best')
            title('Angular Rates', 'FontSize', 20)


            figure(5); clf % Control inputs
            plot(t, rollCmd)
            hold on
            plot(t,pitchCmd)
            plot(t,yawCmd)
            xlabel('Time(s)', 'FontSize', 15)
            ylabel('Control Inputs', 'FontSize', 15)
            legend('Roll Command', 'Pitch Command', 'Yaw Command', 'FontSize', 18, 'Location', 'best')
            title('Control Inputs', 'FontSize', 20)


            figure(6); clf % Actuator states
            plot(t,actuator1)
            hold on
            plot(t,actuator2)
            xlabel('Time(s)', 'FontSize', 15)
            ylabel('Actuator Deflections', 'FontSize', 15)
            legend('A1', 'A2', 'FontSize', 18, 'Location', 'best')
            title('Actuator Deflections', 'FontSize', 20)
        end
    end
end