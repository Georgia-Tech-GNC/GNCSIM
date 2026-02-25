% Main Authors: Karsten Caillet (kcaillet3@gatech.edu)

classdef Analysis < handle

    properties
        numSamples (1,1) double {mustBeNonnegative}
        logs cell;  % Cell array of Log objects
        tStart (1,1) double {mustBeNonnegative}  % Start time for plotting
        tEnd (1,1) double {mustBeNonnegative}    % End time for plotting
        timeMask cell;  % Logical mask for time bounds (applied to first log)
        plotDir (1,1) string;  % Directory to save plots for PDF generation
        saveAsPDF (1,1) logical = false;  % If true, save figures without displaying; if false, display normally
    end

    methods
        function obj = Analysis(logs, tStart, tEnd)
            % Constructor accepts either a single Log object or cell array of Log objects
            % logs: Log object OR {Log1, Log2, ..., LogN}
            % tStart: (optional) Start time for plotting, defaults to minimum time
            % tEnd: (optional) End time for plotting, defaults to maximum time

            % Handle single Log object input
            if isa(logs, 'analysisPkg.Log')
                obj.logs = {logs};  % Convert to cell array
                obj.numSamples = 1;
            else
                obj.logs = logs;
                obj.numSamples = length(logs);
            end

            % Validate that all logs are Log objects
            for i = 1:obj.numSamples
                if ~isa(obj.logs{i}, 'analysisPkg.Log')
                    error('All inputs must be Log objects');
                end
            end

            % Set time bounds
            if nargin < 2 || isempty(tStart)
                obj.tStart = obj.logs{1}.time(1);
            else
                obj.tStart = tStart;
            end

            if nargin < 3 || isempty(tEnd)
                obj.tEnd = obj.logs{1}.time(obj.logs{1}.currentIdx);
            else
                obj.tEnd = tEnd;
            end

            % Compute time mask based on first log
            obj.computeTimeMask();

            % Set default plot directory
            obj.plotDir = tempdir;
        end

        function computeTimeMask(obj)
            % Compute logical mask for time bounds for each log
            obj.timeMask = cell(obj.numSamples, 1);

            if obj.numSamples > 0
                % Use first log to clamp time bounds to available data
                time = obj.logs{1}.time(1:obj.logs{1}.currentIdx);
                actualTStart = max(obj.tStart, time(1));
                actualTEnd = min(obj.tEnd, time(end));

                % Update bounds if they were clamped
                obj.tStart = actualTStart;
                obj.tEnd = actualTEnd;

                % Create mask for each log
                for i = 1:obj.numSamples
                    time = obj.logs{i}.time(1:obj.logs{i}.currentIdx);
                    obj.timeMask{i} = (time >= obj.tStart) & (time <= obj.tEnd);
                end
            end
        end

        function generateTrajectoryPlot(obj)
            fig = figure('Visible', 'off');

            plot3(0,0,0,'o','MarkerSize',10)
            hold on

            for i = 1:obj.numSamples
                log = obj.logs{i};

                % Get position states and apply mask
                x_pos = log.getState('x');
                y_pos = log.getState('y');
                z_pos = log.getState('z');

                x_pos = x_pos(obj.timeMask{i});
                y_pos = y_pos(obj.timeMask{i});
                z_pos = z_pos(obj.timeMask{i});

                % Plot final position
                if ~isempty(x_pos)
                    plot3(x_pos(end), y_pos(end), z_pos(end), 'x','MarkerSize', 10)
                    % Plot trajectory
                    plot3(x_pos, y_pos, z_pos, 'LineWidth', 2)
                end
            end

            grid on
            xlabel('\textbf{Flat Earth X (m)}', 'Interpreter', 'latex')
            ylabel('\textbf{Flat Earth Y (m)}', 'Interpreter', 'latex')
            zlabel('\textbf{Flat Earth Z (m)}', 'Interpreter', 'latex')
            title('\textbf{Trajectory}', 'Interpreter', 'latex')
            axis equal

            if obj.saveAsPDF
                saveas(fig, fullfile(obj.plotDir, 'trajectory.png'));
                close(fig);
            else
                set(fig, 'Visible', 'on');
            end
        end

         function generateVelocityPlot(obj)
            % Calculate Mach Numbers
            a = 340; % m/s (Speed of sound PLACEHOLDER)
            machMax = zeros(obj.numSamples, 1);
            altMax = zeros(obj.numSamples, 1);

            fig = figure('Visible', 'off');
            hold on;

            for i = 1:obj.numSamples
                log = obj.logs{i};

                % Get velocity components
                vx = log.getState('vx');
                vy = log.getState('vy');
                vz = log.getState('vz');
                time = log.time(1:log.currentIdx);
                vx = vx(obj.timeMask{i});
                vy = vy(obj.timeMask{i});
                vz = vz(obj.timeMask{i});
                time = time(obj.timeMask{i});

                plot(time, vx, 'LineWidth', 2)
                plot(time, vy, 'LineWidth', 2)
                plot(time, vz, 'LineWidth', 2)

                % Calculate Mach number
                velMag = sqrt(vx.^2 + vy.^2 + vz.^2);
                machNumbers = velMag ./ a;
                if ~isempty(machNumbers)
                    machMax(i) = max(machNumbers);
                end

                % Get altitude (z position)
                z_pos = log.getState('z');
                z_pos = z_pos(obj.timeMask{i});
                if ~isempty(z_pos)
                    altMax(i) = max(z_pos);
                end
            end

            averageMaxMachNumber = mean(machMax);
            disp(['Average Maximum Mach Number from all flights: ', num2str(averageMaxMachNumber)]);

            averageMaxAltitude = mean(altMax);
            disp(['Average maximum altitude from all flights: ', num2str(averageMaxAltitude),'m']);

            legend('v_x','v_y','v_z')
            ylabel('\textbf{Velocity in Body Frame (m/s)}', 'Interpreter', 'latex')
            xlabel('\textbf{Time (s)}', 'Interpreter', 'latex')
            title('\textbf{Velocity Components}', 'Interpreter', 'latex')
            grid on;

            if obj.saveAsPDF
                saveas(fig, fullfile(obj.plotDir, 'velocity.png'));
                close(fig);
            else
                set(fig, 'Visible', 'on');
            end
        end

        function generateEulerAnglePlot(obj)
            fig = figure('Visible', 'off');
            hold on;

            for i = 1:obj.numSamples
                log = obj.logs{i};

                % Get quaternion states
                q0 = log.getState('q0');
                q1 = log.getState('q1');
                q2 = log.getState('q2');
                q3 = log.getState('q3');
                time = log.time(1:log.currentIdx);
                q0 = q0(obj.timeMask{i});
                q1 = q1(obj.timeMask{i});
                q2 = q2(obj.timeMask{i});
                q3 = q3(obj.timeMask{i});
                time = time(obj.timeMask{i});

                quat = quaternion([q0', q1', q2', q3']);  % N x 4 matrix
                quat_y90 = quaternion(Utils.eulerToQuat([0, 90, 0])');
                quat_rotated = quat * quat_y90;

                eul = rad2deg(quat2eul(quat_rotated, 'XYZ'));
                plot(time, eul, 'LineWidth', 2)
            end


            legend('Yaw','Pitch','Roll')
            ylabel('\textbf{Euler Angles (deg)}', 'Interpreter', 'latex')
            xlabel('\textbf{Time (s)}', 'Interpreter', 'latex')
            title('\textbf{Euler Angles}', 'Interpreter', 'latex')
            grid on

            if obj.saveAsPDF
                saveas(fig, fullfile(obj.plotDir, 'euler_angles.png'));
                close(fig);
            else
                set(fig, 'Visible', 'on');
            end
        end

        function generateYawAnglePlots(obj)
            fig1 = figure('Visible', 'off');
            hold on;
            nominalLog = obj.logs{obj.numSamples};
            timeNominal0 = nominalLog.time(1:nominalLog.currentIdx);
            timeNominal0 = timeNominal0(obj.timeMask{obj.numSamples});
            allEul = zeros(obj.numSamples-1, length(timeNominal0));
            for i = 1:obj.numSamples
                log = obj.logs{i};

                % Get quaternion states
                q0 = log.getState('q0');
                q1 = log.getState('q1');
                q2 = log.getState('q2');
                q3 = log.getState('q3');
                time = log.time(1:log.currentIdx);
                q0 = q0(obj.timeMask{i});
                q1 = q1(obj.timeMask{i});
                q2 = q2(obj.timeMask{i});
                q3 = q3(obj.timeMask{i});
                time = time(obj.timeMask{i});

                quat = quaternion([q0', q1', q2', q3']);  % N x 4 matrix
                quat_y90 = quaternion(Utils.eulerToQuat([0, 90, 0])');
                quat_rotated = quat * quat_y90;

                eul = rad2deg(quat2eul(quat_rotated, 'XYZ'));
                
                % Plot last log (nominal) in red, others (MC) in blue
                if i == obj.numSamples
                    % Last log is nominal flight
                    timeNominal = time;
                    eulNominal   = eul(:,1);
                else
                    % MC runs in light blue
                    if i == 1
                        plot(time, eul(:,1), 'Color', [0.3 0.6 0.9 0.4], 'LineWidth', 0.8, 'DisplayName', 'MC runs')
                    else
                        plot(time, eul(:,1), 'Color', [0.3 0.6 0.9 0.4], 'LineWidth', 0.8, 'HandleVisibility', 'off')
                    end
                    allEul(i, :) = interp1(time, eul(:,1)', timeNominal0, 'linear','extrap')';
                end
            end

            %mu    = mean(allEul, 1, 'omitnan');
            sigma = std(allEul, 0, 1, 'omitnan');
            %eulNominalInterp = interp1(timeNominal, eulNominal, timeNominal0, 'linear');
            
            upper3sigma = eulNominal' + 3*sigma;
            lower3sigma = eulNominal' - 3*sigma;
            
            % Define mission requirements 
            t1 = 2.5;
            t2 = 2.5;
            t3 = 5.7;
            t4 = 5.7;
            y1 = 6.5;
            y2 = 14;
            y3 = 23; 
            y4 = 17.5;
            
            % Draw rectangle
            fill([t1, t2, t3, t4], ...
                 [y1, y2, y3, y4], ...
                 'g', 'FaceAlpha', 0.15, 'EdgeColor', 'g', ...
                 'LineStyle', '--', 'DisplayName', ' Mission Requirement')

            plot(timeNominal0, eulNominal, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Nominal')
            ylabel('Yaw Angle , $(^{\circ})$}', 'Interpreter', 'latex')
            xlabel('Time, s', 'Interpreter', 'latex')
            xlim([0 25])
            legend('show', 'Location', 'best', 'Interpreter', 'latex')
            grid on

            fig2 = figure('Visible', 'off');
            hold on
            %fill([t; flipud(t)],[u;flipud(l)],'b', 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'DisplayName', '±3\sigma')

            for i = 1:obj.numSamples-1
                if i == 1
                    plot(timeNominal0, allEul(i,:), 'Color', [0.3 0.6 0.9 0.4], 'LineWidth', 1, 'DisplayName', 'MC runs')
                else
                    plot(timeNominal0, allEul(i,:), 'Color', [0.3 0.6 0.9 0.4], 'LineWidth', 1, 'HandleVisibility', 'off')
                end
            end

            plot(timeNominal0, upper3sigma, 'b--', 'LineWidth', 0.5, 'DisplayName', '±3$\sigma$' )
            plot(timeNominal0, lower3sigma, 'b--', 'LineWidth', 0.5, 'HandleVisibility', 'off' )

            plot(timeNominal, eulNominal, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Nominal')

            ylabel('\textbf{Yaw Angle (deg)}', 'Interpreter', 'latex')
            xlabel('\textbf{Time (s)}', 'Interpreter', 'latex')
            xlim([0 25])
            legend('show', 'Location', 'best', 'Interpreter', 'latex')
            grid on

            if obj.saveAsPDF
                saveas(fig1, fullfile(obj.plotDir, 'euler_angles_mc.png'));
                saveas(fig2, fullfile(obj.plotDir, 'euler_angles_sigma.png'));
                close(fig1);
                close(fig2);
            else
                set(fig1, 'Visible', 'on');
                set(fig2, 'Visible', 'on');
            end
        end

        function generateAngularVelocityPlot(obj)
            fig = figure('Visible', 'off');
            hold on;

            for i = 1:obj.numSamples
                log = obj.logs{i};

                wx = log.getState('wx');
                wy = log.getState('wy');
                wz = log.getState('wz');
                time = log.time(1:log.currentIdx);
                wx = wx(obj.timeMask{i});
                wy = wy(obj.timeMask{i});
                wz = wz(obj.timeMask{i});
                time = time(obj.timeMask{i});

                plot(time, rad2deg(wx), 'r', 'LineWidth', 2, 'DisplayName', 'ω_x (deg/s)');
                plot(time, rad2deg(wy), 'g', 'LineWidth', 2, 'DisplayName', 'ω_y (deg/s)');
                plot(time, rad2deg(wz), 'b', 'LineWidth', 2, 'DisplayName', 'ω_z (deg/s)');
            end

            xlabel('\textbf{Time (s)}', 'Interpreter', 'latex');
            ylabel('\textbf{Angular Velocity (deg/s)}', 'Interpreter', 'latex');
            grid on;
            hold off;
            legend('show');
            title('\textbf{Angular Velocity Components}', 'Interpreter', 'latex');

            if obj.saveAsPDF
                saveas(fig, fullfile(obj.plotDir, 'angular_velocity.png'));
                close(fig);
            else
                set(fig, 'Visible', 'on');
            end
        end

        function generateMassPlot(obj)
            fig = figure('Visible', 'off');
            hold on;

            for i = 1:obj.numSamples
                log = obj.logs{i};
                mass = log.getState('mass');
                time = log.time(1:log.currentIdx);
                mass = mass(obj.timeMask{i});
                time = time(obj.timeMask{i});

                plot(time, mass, 'LineWidth', 2)
            end

            grid on;
            xlabel('\textbf{Time (s)}', 'Interpreter', 'latex');
            ylabel('\textbf{Mass (kg)}', 'Interpreter', 'latex');
            title('\textbf{Mass}', 'Interpreter', 'latex');

            if obj.saveAsPDF
                saveas(fig, fullfile(obj.plotDir, 'mass.png'));
                close(fig);
            else
                set(fig, 'Visible', 'on');
            end
        end

        function generateThrustPlot(obj)
            fig = figure('Visible', 'off');
            hold on;

            for i = 1:obj.numSamples
                log = obj.logs{i};
                thrust = log.getThrust();
                time = log.time(1:log.currentIdx);
                thrust = thrust(obj.timeMask{i});
                time = time(obj.timeMask{i});

                plot(time, thrust, 'LineWidth', 2)
            end

            grid on;
            xlabel('\textbf{Time (s)}', 'Interpreter', 'latex');
            ylabel('\textbf{Thrust (N)}', 'Interpreter', 'latex');
            title('\textbf{Thrust}', 'Interpreter', 'latex');

            if obj.saveAsPDF
                saveas(fig, fullfile(obj.plotDir, 'thrust.png'));
                close(fig);
            else
                set(fig, 'Visible', 'on');
            end
        end

        function generateAeroMomentPlot(obj)
            fig = figure('Visible', 'off');
            hold on;

            for i = 1:obj.numSamples
                log = obj.logs{i};

                Mx = log.getAeroMoment('Mx');
                My = log.getAeroMoment('My');
                Mz = log.getAeroMoment('Mz');
                time = log.time(1:log.currentIdx);
                Mx = Mx(obj.timeMask{i});
                My = My(obj.timeMask{i});
                Mz = Mz(obj.timeMask{i});
                time = time(obj.timeMask{i});

                plot(time, Mx, 'LineWidth', 2)
                plot(time, My, 'LineWidth', 2)
                plot(time, Mz, 'LineWidth', 2)
            end

            legend('Mx','My','Mz')
            ylabel('\textbf{Aerodynamic Moments (Nm)}', 'Interpreter', 'latex')
            xlabel('\textbf{Time (s)}', 'Interpreter', 'latex')
            title('\textbf{Aerodynamic Moments}', 'Interpreter', 'latex')
            grid on;

            if obj.saveAsPDF
                saveas(fig, fullfile(obj.plotDir, 'aero_moments.png'));
                close(fig);
            else
                set(fig, 'Visible', 'on');
            end
        end

        function generateAeroForcePlot(obj)
            fig = figure('Visible', 'off');
            hold on;

            for i = 1:obj.numSamples
                log = obj.logs{i};

                Fx = log.getAeroForce('Fx');
                Fy = log.getAeroForce('Fy');
                Fz = log.getAeroForce('Fz');
                time = log.time(1:log.currentIdx);
                Fx = Fx(obj.timeMask{i});
                Fy = Fy(obj.timeMask{i});
                Fz = Fz(obj.timeMask{i});
                time = time(obj.timeMask{i});

                plot(time, Fx, 'LineWidth', 2)
                plot(time, Fy, 'LineWidth', 2)
                plot(time, Fz, 'LineWidth', 2)
            end

            legend('Fx','Fy','Fz')
            ylabel('\textbf{Aerodynamic Forces (N)}', 'Interpreter', 'latex')
            xlabel('\textbf{Time (s)}', 'Interpreter', 'latex')
            title('\textbf{Aerodynamic Forces}', 'Interpreter', 'latex')
            grid on;

            if obj.saveAsPDF
                saveas(fig, fullfile(obj.plotDir, 'aero_forces.png'));
                close(fig);
            else
                set(fig, 'Visible', 'on');
            end
        end

        function generateControlInputPlot(obj)
            fig = figure('Visible', 'off');
            hold on;

            for i = 1:obj.numSamples
                log = obj.logs{i};

                for j = 1:length(log.controlInputNames)
                    u = log.getControlInput(log.controlInputNames{j});
                    time = log.time(1:log.currentIdx);
                    u = u(obj.timeMask{i});
                    time = time(obj.timeMask{i});
                    plot(time, u, 'LineWidth', 2)
                end
            end
            
            if ~isempty(obj.logs{1}.controlInputNames)
                legend(obj.logs{1}.controlInputNames)
            end

            ylabel('\textbf{Control Inputs}', 'Interpreter', 'latex')
            xlabel('\textbf{Time (s)}', 'Interpreter', 'latex')
            title('\textbf{Control Inputs}', 'Interpreter', 'latex')
            grid on;

            if obj.saveAsPDF
                saveas(fig, fullfile(obj.plotDir, 'control_inputs.png'));
                close(fig);
            else
                set(fig, 'Visible', 'on');
            end
        end

        function generateActuatorStatePlot(obj)
            fig = figure('Visible', 'off');
            hold on;

            for i = 1:obj.numSamples
                log = obj.logs{i};

                time = log.time(1:log.currentIdx);
                time = time(obj.timeMask{i});

                % Plot all actuator states
                for j = 1:length(log.actuatorStateNames)
                    actuatorData = log.getActuatorState(log.actuatorStateNames{j});
                    actuatorData = actuatorData(obj.timeMask{i});
                    plot(time, actuatorData, 'LineWidth', 2)
                end
            end

            if ~isempty(obj.logs{1}.actuatorStateNames)
                legend(obj.logs{1}.actuatorStateNames)
            end
            ylabel('\textbf{Actuator States}', 'Interpreter', 'latex')
            xlabel('\textbf{Time (s)}', 'Interpreter', 'latex')
            title('\textbf{Actuator States}', 'Interpreter', 'latex')
            grid on;

            if obj.saveAsPDF
                saveas(fig, fullfile(obj.plotDir, 'actuator_states.png'));
                close(fig);
            else
                set(fig, 'Visible', 'on');
            end
        end

        function generateActuatorForcePlot(obj)
            fig = figure('Visible', 'off');
            hold on;

            for i = 1:obj.numSamples
                log = obj.logs{i};

                Fx = log.getActuatorForce('Fx');
                Fy = log.getActuatorForce('Fy');
                Fz = log.getActuatorForce('Fz');
                time = log.time(1:log.currentIdx);
                Fx = Fx(obj.timeMask{i});
                Fy = Fy(obj.timeMask{i});
                Fz = Fz(obj.timeMask{i});
                time = time(obj.timeMask{i});

                plot(time, Fx, 'LineWidth', 2)
                plot(time, Fy, 'LineWidth', 2)
                plot(time, Fz, 'LineWidth', 2)
            end

            legend('Fx','Fy','Fz')
            ylabel('\textbf{Actuator Forces (N)}', 'Interpreter', 'latex')
            xlabel('\textbf{Time (s)}', 'Interpreter', 'latex')
            title('\textbf{Actuator Forces}', 'Interpreter', 'latex')
            grid on;

            if obj.saveAsPDF
                saveas(fig, fullfile(obj.plotDir, 'actuator_forces.png'));
                close(fig);
            else
                set(fig, 'Visible', 'on');
            end
        end

        function generateActuatorMomentPlot(obj)
            fig = figure('Visible', 'off');
            hold on;

            for i = 1:obj.numSamples
                log = obj.logs{i};

                Mx = log.getActuatorMoment('Mx');
                My = log.getActuatorMoment('My');
                Mz = log.getActuatorMoment('Mz');
                time = log.time(1:log.currentIdx);
                Mx = Mx(obj.timeMask{i});
                My = My(obj.timeMask{i});
                Mz = Mz(obj.timeMask{i});
                time = time(obj.timeMask{i});

                plot(time, Mx, 'LineWidth', 2)
                plot(time, My, 'LineWidth', 2)
                plot(time, Mz, 'LineWidth', 2)
            end

            legend('Mx','My','Mz')
            ylabel('\textbf{Actuator Moments (Nm)}', 'Interpreter', 'latex')
            xlabel('\textbf{Time (s)}', 'Interpreter', 'latex')
            title('\textbf{Actuator Moments}', 'Interpreter', 'latex')
            grid on;

            if obj.saveAsPDF
                saveas(fig, fullfile(obj.plotDir, 'actuator_moments.png'));
                close(fig);
            else
                set(fig, 'Visible', 'on');
            end
        end

        function generateAccelerationPlot(obj)
            fig = figure('Visible', 'off');
            hold on;

            for i = 1:obj.numSamples
                log = obj.logs{i};

                % Get acceleration components (derivatives of velocities)
                ax = log.getStateDot('vx');
                ay = log.getStateDot('vy');
                az = log.getStateDot('vz');
                time = log.time(1:log.currentIdx);
                ax = ax(obj.timeMask{i});
                ay = ay(obj.timeMask{i});
                az = az(obj.timeMask{i});
                time = time(obj.timeMask{i});

                plot(time, ax, 'LineWidth', 2)
                plot(time, ay, 'LineWidth', 2)
                plot(time, az, 'LineWidth', 2)
            end

            legend('a_x','a_y','a_z')
            ylabel('\textbf{Acceleration in Body Frame (m/s$^2$)}', 'Interpreter', 'latex')
            xlabel('\textbf{Time (s)}', 'Interpreter', 'latex')
            title('\textbf{Acceleration Components}', 'Interpreter', 'latex')
            grid on;

            if obj.saveAsPDF
                saveas(fig, fullfile(obj.plotDir, 'acceleration.png'));
                close(fig);
            else
                set(fig, 'Visible', 'on');
            end
        end

        function generateEkfCovariancePlot(obj)
            % Plot EKF covariances for position and velocity
            % Creates a figure with 2 subplots: one for position (Px, Py, Pz)
            % and one for velocity (Vx, Vy, Vz)

            fig = figure('Visible', 'off');

            % Subplot 1: Position Covariances
            subplot(2, 1, 1);
            hold on;

            for i = 1:obj.numSamples
                log = obj.logs{i};

                Px = log.getEkfCovariance('Px');
                Py = log.getEkfCovariance('Py');
                Pz = log.getEkfCovariance('Pz');
                time = log.time(1:log.currentIdx);
                Px = Px(obj.timeMask{i});
                Py = Py(obj.timeMask{i});
                Pz = Pz(obj.timeMask{i});
                time = time(obj.timeMask{i});

                plot(time, Px, 'LineWidth', 2)
                plot(time, Py, 'LineWidth', 2)
                plot(time, Pz, 'LineWidth', 2)
            end

            legend('P_x', 'P_y', 'P_z')
            ylabel('\textbf{Position Covariance (m$^2$)}', 'Interpreter', 'latex')
            xlabel('\textbf{Time (s)}', 'Interpreter', 'latex')
            title('\textbf{EKF Position Covariances}', 'Interpreter', 'latex')
            grid on;
            hold off;

            % Subplot 2: Velocity Covariances
            subplot(2, 1, 2);
            hold on;

            for i = 1:obj.numSamples
                log = obj.logs{i};

                Vx = log.getEkfCovariance('Vx');
                Vy = log.getEkfCovariance('Vy');
                Vz = log.getEkfCovariance('Vz');
                time = log.time(1:log.currentIdx);
                Vx = Vx(obj.timeMask{i});
                Vy = Vy(obj.timeMask{i});
                Vz = Vz(obj.timeMask{i});
                time = time(obj.timeMask{i});

                plot(time, Vx, 'LineWidth', 2)
                plot(time, Vy, 'LineWidth', 2)
                plot(time, Vz, 'LineWidth', 2)
            end

            legend('V_x', 'V_y', 'V_z')
            ylabel('\textbf{Velocity Covariance ((m/s)$^2$)}', 'Interpreter', 'latex')
            xlabel('\textbf{Time (s)}', 'Interpreter', 'latex')
            title('\textbf{EKF Velocity Covariances}', 'Interpreter', 'latex')
            grid on;
            hold off;

            if obj.saveAsPDF
                saveas(fig, fullfile(obj.plotDir, 'ekf_covariance.png'));
                close(fig);
            else
                set(fig, 'Visible', 'on');
            end
        end

        function generateAltitudeAndXYPlot(obj)
            % Plot altitude, x, and y coordinates as a function of time
            % Creates figures with altitude and X/Y plots

            fig1 = figure('Visible', 'off');
            % Subplot 1: Altitude vs Time
            hold on;

            for i = 1:obj.numSamples
                log = obj.logs{i};
                z_pos = log.getState('z');
                time = log.time(1:log.currentIdx);
                z_pos = z_pos(obj.timeMask{i});
                time = time(obj.timeMask{i});

                plot(time, z_pos, 'LineWidth', 2);
            end

            grid on;
            xlabel('\textbf{Time (s)}', 'Interpreter', 'latex');
            ylabel('\textbf{Altitude z (m)}', 'Interpreter', 'latex');
            title('\textbf{Altitude vs Time}', 'Interpreter', 'latex');
            hold off;

            if obj.saveAsPDF
                saveas(fig1, fullfile(obj.plotDir, 'altitude.png'));
                close(fig1);
            else
                set(fig1, 'Visible', 'on');
            end

            fig2 = figure('Visible', 'off');
            % Subplot 2: X Coordinate vs Time
            subplot(2, 1, 1);
            hold on;

            for i = 1:obj.numSamples
                log = obj.logs{i};
                x_pos = log.getState('x');
                time = log.time(1:log.currentIdx);
                x_pos = x_pos(obj.timeMask{i});
                time = time(obj.timeMask{i});

                plot(time, x_pos, 'LineWidth', 2);
            end

            grid on;
            xlabel('\textbf{Time (s)}', 'Interpreter', 'latex');
            ylabel('\textbf{X Position (m)}', 'Interpreter', 'latex');
            title('\textbf{X Coordinate vs Time}', 'Interpreter', 'latex');
            hold off;

            % Subplot 3: Y Coordinate vs Time
            subplot(2, 1, 2);
            hold on;

            for i = 1:obj.numSamples
                log = obj.logs{i};
                y_pos = log.getState('y');
                time = log.time(1:log.currentIdx);
                y_pos = y_pos(obj.timeMask{i});
                time = time(obj.timeMask{i});

                plot(time, y_pos, 'LineWidth', 2);
            end

            grid on;
            xlabel('\textbf{Time (s)}', 'Interpreter', 'latex');
            ylabel('\textbf{Y Position (m)}', 'Interpreter', 'latex');
            title('\textbf{Y Coordinate vs Time}', 'Interpreter', 'latex');
            hold off;

            if obj.saveAsPDF
                saveas(fig2, fullfile(obj.plotDir, 'position_xy.png'));
                close(fig2);
            else
                set(fig2, 'Visible', 'on');
            end
		end

        function generateParachuteForcePlot(obj)
            fig = figure('Visible', 'off');
            hold on;

            for i = 1:obj.numSamples
                log = obj.logs{i};

                Fx = log.parachuteForces(1, 1:log.currentIdx);
                Fy = log.parachuteForces(2, 1:log.currentIdx);
                Fz = log.parachuteForces(3, 1:log.currentIdx);
                time = log.time(1:log.currentIdx);
                Fx = Fx(obj.timeMask{i});
                Fy = Fy(obj.timeMask{i});
                Fz = Fz(obj.timeMask{i});
                time = time(obj.timeMask{i});

                plot(time, Fx, 'LineWidth', 2)
                plot(time, Fy, 'LineWidth', 2)
                plot(time, Fz, 'LineWidth', 2)
            end

            legend('Fx','Fy','Fz')
            ylabel('\textbf{Parachute Forces (N)}', 'Interpreter', 'latex')
            xlabel('\textbf{Time (s)}', 'Interpreter', 'latex')
            title('\textbf{Parachute Forces}', 'Interpreter', 'latex')
            grid on;

            if obj.saveAsPDF
                saveas(fig, fullfile(obj.plotDir, 'parachute_forces.png'));
                close(fig);
            else
                set(fig, 'Visible', 'on');
            end
        end

        function generateLandingDispersionPlot(obj)
            % Plot landing dispersion - shows final x,y position for each run that landed
            % Each point represents where a rocket landed

            fig = figure('Visible', 'off');
            hold on;

            % Arrays to collect landing positions
            landingX = [];
            landingY = [];

            for i = 1:obj.numSamples
                log = obj.logs{i};

                % Check if flight states are recorded
                if isempty(log.flightStates)
                    warning('Log %d does not have flight state data. Skipping.', i);
                    continue;
                end

                % Get the last recorded flight state
                lastFlightState = log.flightStates(log.currentIdx);

                % Check if the final state was LANDED (FlightState.LANDED = 6)
                if lastFlightState == uint8(flightPkg.FlightState.LANDED)
                    % Get the final x,y position
                    x_pos = log.getState('x');
                    y_pos = log.getState('y');

                    landingX(end+1) = x_pos(end);
                    landingY(end+1) = y_pos(end);
                end
            end

            % Plot landing positions
            if ~isempty(landingX)
                plot(landingX, landingY, 'o', 'MarkerSize', 8, 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'k');

                % Plot launch point
                plot(0, 0, 'rx', 'MarkerSize', 15, 'LineWidth', 3);

                % Calculate and display statistics
                meanX = mean(landingX);
                meanY = mean(landingY);
                stdX = std(landingX);
                stdY = std(landingY);

                % Plot mean landing position
                plot(meanX, meanY, 'r+', 'MarkerSize', 15, 'LineWidth', 3);

                % Add text with statistics
                legendEntries = {'Landing Points', 'Launch Point', 'Mean Landing'};
                legend(legendEntries, 'Location', 'best');

                % Add statistics text box
                statsText = sprintf('Landed: %d/%d\nMean X: %.2f m\nMean Y: %.2f m\nStd X: %.2f m\nStd Y: %.2f m', ...
                    length(landingX), obj.numSamples, meanX, meanY, stdX, stdY);
                annotation('textbox', [0.15, 0.75, 0.3, 0.15], 'String', statsText, ...
                    'FitBoxToText', 'on', 'BackgroundColor', 'white', 'EdgeColor', 'black');

                disp(['Landing dispersion statistics:']);
                disp(['  Successful landings: ', num2str(length(landingX)), '/', num2str(obj.numSamples)]);
                disp(['  Mean landing position: (', num2str(meanX), ', ', num2str(meanY), ') m']);
                disp(['  Landing std dev: (', num2str(stdX), ', ', num2str(stdY), ') m']);
            else
                text(0.5, 0.5, 'No landed flights found', 'HorizontalAlignment', 'center');
                disp('Warning: No flights reached LANDED state');
            end

            grid on;
            axis equal;
            xlabel('\textbf{X Position (m)}', 'Interpreter', 'latex');
            ylabel('\textbf{Y Position (m)}', 'Interpreter', 'latex');
            title('\textbf{Landing Dispersion}', 'Interpreter', 'latex');

            if obj.saveAsPDF
                saveas(fig, fullfile(obj.plotDir, 'landing_dispersion.png'));
                close(fig);
            else
                set(fig, 'Visible', 'on');
            end
        end

		function printReliventInfo(obj)
			a = 340; % m/s (Speed of sound PLACEHOLDER)
            for i = 1:obj.numSamples
                log = obj.logs{i};

                % Get velocity component estimates
                vx = log.getState('vx');
                vy = log.getState('vy');
                vz = log.getState('vz');
                vx = vx(obj.timeMask{i});
                vy = vy(obj.timeMask{i});
                vz = vz(obj.timeMask{i});
				time = log.time(1:log.currentIdx);

                % Calculate Mach number
                velMag = sqrt(vx.^2 + vy.^2 + vz.^2);
                machNumbers = velMag ./ a;
                if ~isempty(machNumbers)
                    machMax(i) = max(machNumbers);
					velMax(i) = max(velMag);
                end

                % Get altitude (z position estimate)
                z_pos = log.getState('z');
                z_pos = z_pos(obj.timeMask{i});
                if ~isempty(z_pos)
                    altMax(i) = max(z_pos);
                end
            end

            averageMaxMachNumber = mean(machMax);
            disp(['[Estimate] Average Maximum Mach Number from all flights: ', num2str(averageMaxMachNumber)]);

			averageMaxVel = mean(velMax);
            disp(['[Estimate] Average Maximum Velocity from all flights: ', num2str(averageMaxVel)]);

			try
                % Attempt to use Signal Processing Toolbox
                [~, locs] = findpeaks(z_pos);
                appoIndex = max(locs);
                timeAtAppo = time(appoIndex);
                disp(['[Estimate] Average Time to Apogee (Signal Toolbox): ', num2str(timeAtAppo)]);
            catch
                % Fallback for when the toolbox isn't installed/licensed
                [~, appoIndex] = max(z_pos); 
                timeAtAppo = time(appoIndex);
                warning('Signal Processing Toolbox not found. Falling back to simple maximum for apogee detection.');
                disp(['[Estimate] Time to Max Altitude (Fallback): ', num2str(timeAtAppo)]);
            end

            averageMaxAltitude = mean(altMax);
            disp(['[Estimate] Average maximum altitude from all flights: ', num2str(averageMaxAltitude),' m']);
		end

        function generateAllPlots(obj)
            obj.generateTrajectoryPlot();
            obj.generateVelocityPlot();
            obj.generateAccelerationPlot();
            obj.generateEulerAnglePlot();
            obj.generateAngularVelocityPlot();
            obj.generateMassPlot();
            obj.generateThrustPlot();
            obj.generateAeroMomentPlot();
            obj.generateAeroForcePlot();
            obj.generateControlInputPlot();
            obj.generateActuatorStatePlot();
            obj.generateActuatorForcePlot();
            obj.generateActuatorMomentPlot();
            obj.generateEkfCovariancePlot();
            obj.generateAltitudeAndXYPlot();
            obj.generateParachuteForcePlot();
            obj.generateLandingDispersionPlot();
			obj.printReliventInfo();
        end

        % State Estimate Plotting Functions
        function generateTrajectoryEstimatePlot(obj)
            fig = figure('Visible', 'off');

            plot3(0,0,0,'o','MarkerSize',10)
            hold on

            for i = 1:obj.numSamples
                log = obj.logs{i};

                % Get position state estimates
                x_pos = log.getStateEstimate('x');
                y_pos = log.getStateEstimate('y');
                z_pos = log.getStateEstimate('z');
                time = log.time(1:log.currentIdx);
                x_pos = x_pos(obj.timeMask{i});
                y_pos = y_pos(obj.timeMask{i});
                z_pos = z_pos(obj.timeMask{i});

                % Plot final position
                if ~isempty(x_pos)
                    plot3(x_pos(end), y_pos(end), z_pos(end), 'x','MarkerSize', 10)
                    % Plot trajectory
                    plot3(x_pos, y_pos, z_pos, 'LineWidth', 2)
                end
            end

            grid on
            xlabel('\textbf{Flat Earth X (m)}', 'Interpreter', 'latex')
            ylabel('\textbf{Flat Earth Y (m)}', 'Interpreter', 'latex')
            zlabel('\textbf{Flat Earth Z (m)}', 'Interpreter', 'latex')
            title('\textbf{Trajectory (State Estimate)}', 'Interpreter', 'latex')
            axis equal

            if obj.saveAsPDF
                saveas(fig, fullfile(obj.plotDir, 'trajectory_estimate.png'));
                close(fig);
            else
                set(fig, 'Visible', 'on');
            end
        end

        function generateVelocityEstimatePlot(obj)
            % Calculate Mach Numbers
            a = 340; % m/s (Speed of sound PLACEHOLDER)
            machMax = zeros(obj.numSamples, 1);
            altMax = zeros(obj.numSamples, 1);

            fig = figure('Visible', 'off');
            hold on;

            for i = 1:obj.numSamples
                log = obj.logs{i};

                % Get velocity component estimates
                vx = log.getStateEstimate('vx');
                vy = log.getStateEstimate('vy');
                vz = log.getStateEstimate('vz');
                time = log.time(1:log.currentIdx);
                vx = vx(obj.timeMask{i});
                vy = vy(obj.timeMask{i});
                vz = vz(obj.timeMask{i});
                time = time(obj.timeMask{i});

                plot(time, vx, 'LineWidth', 2)
                plot(time, vy, 'LineWidth', 2)
                plot(time, vz, 'LineWidth', 2)

                % Calculate Mach number
                velMag = sqrt(vx.^2 + vy.^2 + vz.^2);
                machNumbers = velMag ./ a;
                if ~isempty(machNumbers)
                    machMax(i) = max(machNumbers);
                end

                % Get altitude (z position estimate)
                z_pos = log.getStateEstimate('z');
                z_pos = z_pos(obj.timeMask{i});
                if ~isempty(z_pos)
                    altMax(i) = max(z_pos);
                end
            end

            %averageMaxMachNumber = mean(machMax);
            %disp(['[Estimate] Average Maximum Mach Number from all flights: ', num2str(averageMaxMachNumber)]);

            %averageMaxAltitude = mean(altMax);
            %disp(['[Estimate] Average maximum altitude from all flights: ', num2str(averageMaxAltitude),'m']);

            legend('v_x','v_y','v_z')
            ylabel('\textbf{Velocity in Body Frame (m/s)}', 'Interpreter', 'latex')
            xlabel('\textbf{Time (s)}', 'Interpreter', 'latex')
            title('\textbf{Velocity (State Estimate)}', 'Interpreter', 'latex')
            grid on;

            if obj.saveAsPDF
                saveas(fig, fullfile(obj.plotDir, 'velocity_estimate.png'));
                close(fig);
            else
                set(fig, 'Visible', 'on');
            end
        end

        function generateEulerAngleEstimatePlot(obj)
            fig = figure('Visible', 'off');
            hold on;

            for i = 1:obj.numSamples
                log = obj.logs{i};

                % Get quaternion state estimates
                q0 = log.getStateEstimate('q0');
                q1 = log.getStateEstimate('q1');
                q2 = log.getStateEstimate('q2');
                q3 = log.getStateEstimate('q3');
                time = log.time(1:log.currentIdx);
                q0 = q0(obj.timeMask{i});
                q1 = q1(obj.timeMask{i});
                q2 = q2(obj.timeMask{i});
                q3 = q3(obj.timeMask{i});
                time = time(obj.timeMask{i});

                quat = quaternion([q0', q1', q2', q3']);  % N x 4 matrix
                quat_y90 = quaternion(Utils.eulerToQuat([0, 90, 0])');
                quat_rotated = quat * quat_y90;

                eul = rad2deg(quat2eul(quat_rotated, 'ZYX'));
                plot(time, eul(:,1), 'LineWidth', 2)
            end


            ylabel('\textbf{Yaw Angle (deg)}', 'Interpreter', 'latex')
            xlabel('\textbf{Time (s)}', 'Interpreter', 'latex')
            xlim([0,24])
            %title('\textbf{Euler Angles (State Estimate)}', 'Interpreter', 'latex')
            grid on

            if obj.saveAsPDF
                saveas(fig, fullfile(obj.plotDir, 'euler_angles_estimate.png'));
                close(fig);
            else
                set(fig, 'Visible', 'on');
            end
        end

        function generateAngularVelocityEstimatePlot(obj)
            fig = figure('Visible', 'off');
            hold on;

            for i = 1:obj.numSamples
                log = obj.logs{i};

                wx = log.getStateEstimate('wx');
                wy = log.getStateEstimate('wy');
                wz = log.getStateEstimate('wz');
                time = log.time(1:log.currentIdx);
                wx = wx(obj.timeMask{i});
                wy = wy(obj.timeMask{i});
                wz = wz(obj.timeMask{i});
                time = time(obj.timeMask{i});

                plot(time, rad2deg(wx), 'r', 'LineWidth', 2, 'DisplayName', 'ω_x (deg/s)');
                plot(time, rad2deg(wy), 'g', 'LineWidth', 2, 'DisplayName', 'ω_y (deg/s)');
                plot(time, rad2deg(wz), 'b', 'LineWidth', 2, 'DisplayName', 'ω_z (deg/s)');
            end

            xlabel('\textbf{Time (s)}', 'Interpreter', 'latex');
            ylabel('\textbf{Angular Velocity (rad/s)}', 'Interpreter', 'latex');
            grid on;
            hold off;
            legend('show');
            title('\textbf{Angular Velocity (State Estimate)}', 'Interpreter', 'latex');

            if obj.saveAsPDF
                saveas(fig, fullfile(obj.plotDir, 'angular_velocity_estimate.png'));
                close(fig);
            else
                set(fig, 'Visible', 'on');
            end
        end

        function generateAltitudeAndXYEstimatePlot(obj)
            % Plot altitude, x, and y coordinate estimates as a function of time

            fig1 = figure('Visible', 'off');
            % Altitude vs Time
            hold on;

            for i = 1:obj.numSamples
                log = obj.logs{i};
                z_pos = log.getStateEstimate('z');
                time = log.time(1:log.currentIdx);
                z_pos = z_pos(obj.timeMask{i});
                time = time(obj.timeMask{i});

                plot(time, z_pos, 'LineWidth', 2);
            end

            grid on;
            xlabel('\textbf{Time (s)}', 'Interpreter', 'latex');
            ylabel('\textbf{Altitude z (m)}', 'Interpreter', 'latex');
            title('\textbf{Altitude vs Time (State Estimate)}', 'Interpreter', 'latex');
            hold off;

            if obj.saveAsPDF
                saveas(fig1, fullfile(obj.plotDir, 'altitude_estimate.png'));
                close(fig1);
            else
                set(fig1, 'Visible', 'on');
            end

            fig2 = figure('Visible', 'off');
            % X Coordinate vs Time
            subplot(2, 1, 1);
            hold on;

            for i = 1:obj.numSamples
                log = obj.logs{i};
                x_pos = log.getStateEstimate('x');
                time = log.time(1:log.currentIdx);
                x_pos = x_pos(obj.timeMask{i});
                time = time(obj.timeMask{i});

                plot(time, x_pos, 'LineWidth', 2);
            end

            grid on;
            xlabel('\textbf{Time (s)}', 'Interpreter', 'latex');
            ylabel('\textbf{X Position (m)}', 'Interpreter', 'latex');
            title('\textbf{X Coordinate vs Time (State Estimate)}', 'Interpreter', 'latex');
            hold off;

            % Y Coordinate vs Time
            subplot(2, 1, 2);
            hold on;

            for i = 1:obj.numSamples
                log = obj.logs{i};
                y_pos = log.getStateEstimate('y');
                time = log.time(1:log.currentIdx);
                y_pos = y_pos(obj.timeMask{i});
                time = time(obj.timeMask{i});

                plot(time, y_pos, 'LineWidth', 2);
            end

            grid on;
            xlabel('\textbf{Time (s)}', 'Interpreter', 'latex');
            ylabel('\textbf{Y Position (m)}', 'Interpreter', 'latex');
            title('\textbf{Y Coordinate vs Time (State Estimate)}', 'Interpreter', 'latex');
            hold off;

            if obj.saveAsPDF
                saveas(fig2, fullfile(obj.plotDir, 'position_xy_estimate.png'));
                close(fig2);
            else
                set(fig2, 'Visible', 'on');
            end
        end

        function generateAllEstimatePlots(obj)
            obj.generateTrajectoryEstimatePlot();
            obj.generateVelocityEstimatePlot();
            obj.generateEulerAngleEstimatePlot();
            obj.generateAngularVelocityEstimatePlot();
            obj.generateAltitudeAndXYEstimatePlot();
        end

        % Additional helper methods
        function time = getCommonTime(obj)
            % Returns time vector from first log (assumes all synchronized)
            if obj.numSamples > 0
                time = obj.logs{1}.time(1:obj.logs{1}.currentIdx);
                % Apply time bounds
                time = time(obj.timeMask{1});
            else
                time = [];
            end
        end

        function stats = computeStatistics(obj, stateName)
            arguments
                obj
                stateName (1,1) string
            end
            % Compute mean and std dev across all samples for a given state
            nPoints = sum(obj.timeMask{1});
            data = zeros(nPoints, obj.numSamples);

            for i = 1:obj.numSamples
                fullData = obj.logs{i}.getState(stateName);
                data(:, i) = fullData(obj.timeMask{i});
            end

            stats.mean = mean(data, 2);
            stats.std = std(data, 0, 2);
            stats.min = min(data, [], 2);
            stats.max = max(data, [], 2);
        end

        function plotStateStatistics(obj, stateName)
            arguments
                obj
                stateName (1,1) string
            end
            % Plot mean ± std for a given state
            stats = obj.computeStatistics(stateName);
            time = obj.getCommonTime();

            figure;
            hold on;

            % Plot mean
            plot(time, stats.mean, 'b-', 'LineWidth', 2, 'DisplayName', 'Mean');

            % Plot confidence interval
            fill([time, fliplr(time)], ...
                 [stats.mean + stats.std; flipud(stats.mean - stats.std)], ...
                 'b', 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'DisplayName', '\pm 1\sigma');

            % Plot min/max envelope
            plot(time, stats.min, 'r--', 'LineWidth', 1, 'DisplayName', 'Min/Max');
            plot(time, stats.max, 'r--', 'LineWidth', 1, 'HandleVisibility', 'off');

            xlabel('\textbf{Time (s)}', 'Interpreter', 'latex');
            ylabel(['\textbf{', strrep(stateName, '_', '\_'), '}'], 'Interpreter', 'latex');
            legend('show', 'Interpreter', 'latex');
            grid on;
            title(['\textbf{', strrep(stateName, '_', '\_'), ' Statistics}'], 'Interpreter', 'latex');
        end

        function generatePdfFile(obj, filename)
            arguments
                obj
                filename (1,1) string = 'Flight_Analysis_Report'
            end
            % Generate PDF report using report generator toolbox
            % filename: (optional) Output filename without extension, defaults to 'Flight_Analysis_Report'

            if nargin < 2
                filename = 'Flight_Analysis_Report';
            end

            % Import report generator classes
            import mlreportgen.report.*
            import mlreportgen.dom.*

            % Set plot directory to temp
            obj.plotDir = tempdir;

            % Enable PDF mode - figures will be saved without displaying
            obj.saveAsPDF = true;

            % Generate all plots (saved to plotDir)
            obj.generateAllPlots();

            % Create the report
            rpt = Report(fullfile(pwd, filename), "pdf");

            % Title Page
            tp = TitlePage;
            tp.Title = "Flight Analysis Report";
            tp.Subtitle = sprintf('Time Range: %.2f s to %.2f s', obj.tStart, obj.tEnd);
            tp.Author = "GNC Controls Team";
            add(rpt, tp);

            % Section 1: Analysis Parameters
            sec1 = Section("Analysis Parameters");
            para1 = Paragraph(sprintf('Number of samples: %d', obj.numSamples));
            add(sec1, para1);
            para2 = Paragraph(sprintf('Start time: %.4f s', obj.tStart));
            add(sec1, para2);
            para3 = Paragraph(sprintf('End time: %.4f s', obj.tEnd));
            add(sec1, para3);
            para4 = Paragraph(sprintf('Time duration: %.4f s', obj.tEnd - obj.tStart));
            add(sec1, para4);
            add(rpt, sec1);

            % Section 2: Trajectory Analysis
            sec2 = Section("Trajectory Analysis");

            if isfile(fullfile(obj.plotDir, 'trajectory.png'))
                img = Image(fullfile(obj.plotDir, 'trajectory.png'));
                img.Width = "6in";
                img.Height = "4in";
                add(sec2, img);
            end

            if isfile(fullfile(obj.plotDir, 'altitude.png'))
                img = Image(fullfile(obj.plotDir, 'altitude.png'));
                img.Width = "6in";
                img.Height = "4in";
                add(sec2, img);
            end

            if isfile(fullfile(obj.plotDir, 'position_xy.png'))
                img = Image(fullfile(obj.plotDir, 'position_xy.png'));
                img.Width = "6in";
                img.Height = "4in";
                add(sec2, img);
            end

            add(rpt, sec2);

            % Section 3: Velocity and Acceleration
            sec3 = Section("Velocity and Acceleration Analysis");

            if isfile(fullfile(obj.plotDir, 'velocity.png'))
                img = Image(fullfile(obj.plotDir, 'velocity.png'));
                img.Width = "6in";
                img.Height = "4in";
                add(sec3, img);
            end

            if isfile(fullfile(obj.plotDir, 'acceleration.png'))
                img = Image(fullfile(obj.plotDir, 'acceleration.png'));
                img.Width = "6in";
                img.Height = "4in";
                add(sec3, img);
            end

            add(rpt, sec3);

            % Section 4: Attitude Analysis
            sec4 = Section("Attitude Analysis");

            if isfile(fullfile(obj.plotDir, 'euler_angles.png'))
                img = Image(fullfile(obj.plotDir, 'euler_angles.png'));
                img.Width = "6in";
                img.Height = "4in";
                add(sec4, img);
            end

            if isfile(fullfile(obj.plotDir, 'angular_velocity.png'))
                img = Image(fullfile(obj.plotDir, 'angular_velocity.png'));
                img.Width = "6in";
                img.Height = "4in";
                add(sec4, img);
            end

            add(rpt, sec4);

            % Section 5: Propulsion and Mass
            sec5 = Section("Propulsion and Mass");

            if isfile(fullfile(obj.plotDir, 'thrust.png'))
                img = Image(fullfile(obj.plotDir, 'thrust.png'));
                img.Width = "6in";
                img.Height = "4in";
                add(sec5, img);
            end

            if isfile(fullfile(obj.plotDir, 'mass.png'))
                img = Image(fullfile(obj.plotDir, 'mass.png'));
                img.Width = "6in";
                img.Height = "4in";
                add(sec5, img);
            end

            add(rpt, sec5);

            % Section 6: Moments Analysis
            sec6 = Section("Moments Analysis");

            if isfile(fullfile(obj.plotDir, 'aero_moments.png'))
                img = Image(fullfile(obj.plotDir, 'aero_moments.png'));
                img.Width = "6in";
                img.Height = "4in";
                add(sec6, img);
            end

            if isfile(fullfile(obj.plotDir, 'control_moments.png'))
                img = Image(fullfile(obj.plotDir, 'control_moments.png'));
                img.Width = "6in";
                img.Height = "4in";
                add(sec6, img);
            end

            add(rpt, sec6);

            % Section 7: EKF Covariance
            sec7 = Section("EKF Covariance Analysis");

            if isfile(fullfile(obj.plotDir, 'ekf_covariance.png'))
                img = Image(fullfile(obj.plotDir, 'ekf_covariance.png'));
                img.Width = "6in";
                img.Height = "4in";
                add(sec7, img);
            end

            add(rpt, sec7);

            % Reset saveAsPDF flag to default
            obj.saveAsPDF = false;

            % Close and view report
            close(rpt);
            rptview(rpt);
        end
    end
end