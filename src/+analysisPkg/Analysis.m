classdef Analysis < handle
    
    properties
        numSamples;
        logs;  % Cell array of Log objects
    end

    methods
        function obj = Analysis(logs)
            % Constructor accepts either a single Log object or cell array of Log objects
            % logs: Log object OR {Log1, Log2, ..., LogN}
            
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
        end

        function generatePdfFile(obj)
            % Not needed here yet (complete in 34-branch)
        end

        function generateTrajectoryPlot(obj)
            figure;

            plot3(0,0,0,'o','MarkerSize',10)
            hold on
            
            for i = 1:obj.numSamples
                log = obj.logs{i};
                
                % Get position states (assuming indices 2:4 are x,y,z)
                x_pos = log.getState('x');
                y_pos = log.getState('y');
                z_pos = log.getState('z');
                
                % Plot final position
                plot3(x_pos(end), y_pos(end), z_pos(end), 'x','MarkerSize', 10)
                % Plot trajectory
                plot3(x_pos, y_pos, z_pos, 'LineWidth', 2)
            end
            
            grid on
            xlabel('\textbf{Flat Earth X (m)}', 'Interpreter', 'latex')
            ylabel('\textbf{Flat Earth Y (m)}', 'Interpreter', 'latex')
            zlabel('\textbf{Flat Earth Z (m)}', 'Interpreter', 'latex')
            axis equal
        end

         function generateVelocityPlot(obj)
            % Calculate Mach Numbers
            a = 340; % m/s (Speed of sound PLACEHOLDER)
            machMax = zeros(obj.numSamples, 1);
            altMax = zeros(obj.numSamples, 1);

            figure;
            hold on;
            
            for i = 1:obj.numSamples
                log = obj.logs{i};
                
                % Get velocity components
                vx = log.getState('vx');
                vy = log.getState('vy');
                vz = log.getState('vz');

                time = log.time;
                plot(time, vx, 'LineWidth', 2)
                plot(time, vy, 'LineWidth', 2)
                plot(time, vz, 'LineWidth', 2)
                
                % Calculate Mach number
                velMag = sqrt(vx.^2 + vy.^2 + vz.^2);
                machNumbers = velMag ./ a;
                machMax(i) = max(machNumbers);
                
                % Get altitude (x position)
                z_pos = log.getState('z');
                altMax(i) = max(z_pos);
            end
            
            averageMaxMachNumber = mean(machMax);
            disp(['Average Maximum Mach Number from all flights: ', num2str(averageMaxMachNumber)]);
            
            averageMaxAltitude = mean(altMax);
            disp(['Average maximum altitude from all flights: ', num2str(averageMaxAltitude),'m']);

            legend('v_x','v_y','v_z')
            ylabel('\textbf{Velocity in Body Frame (m/s)}', 'Interpreter', 'latex')
            xlabel('\textbf{Time (s)}', 'Interpreter', 'latex')
            grid on;
        end

        function generateEulerAnglePlot(obj)
            figure;
            hold on;

            for i = 1:obj.numSamples
                log = obj.logs{i};
                
                % Get quaternion states
                q0 = log.getState('q0');
                q1 = log.getState('q1');
                q2 = log.getState('q2');
                q3 = log.getState('q3');
                
                quat = quaternion([q0', q1', q2', q3']);  % N x 4 matrix
                quat_y90 = quaternion(Utils.eulerToQuat([0, 90, 0])');
                quat_rotated = quat * quat_y90;
                
                eul = rad2deg(quat2eul(quat_rotated, 'ZYX'));  
                plot(log.time, eul, 'LineWidth', 2)
            end
 
            
            legend('Yaw','Pitch','Roll')
            ylabel('\textbf{Euler Angles (deg)}', 'Interpreter', 'latex')
            xlabel('\textbf{Time (s)}', 'Interpreter', 'latex')
            grid on
        end

        function generateAngularVelocityPlot(obj)
            figure;
            hold on;
            
            for i = 1:obj.numSamples
                log = obj.logs{i};
                
                wx = log.getState('wx');
                wy = log.getState('wy');
                wz = log.getState('wz');
                
                time = log.time(1:log.currentIdx);
                
                plot(time, rad2deg(wx), 'r', 'LineWidth', 2, 'DisplayName', 'ω_x (deg/s)');
                plot(time, rad2deg(wy), 'g', 'LineWidth', 2, 'DisplayName', 'ω_y (deg/s)');
                plot(time, rad2deg(wz), 'b', 'LineWidth', 2, 'DisplayName', 'ω_z (deg/s)');
            end
            
            xlabel('\textbf{Time (s)}', 'Interpreter', 'latex');
            ylabel('\textbf{Angular Velocity (rad/s)}', 'Interpreter', 'latex');
            grid on;
            hold off;
            legend('show');
            title('\textbf{Angular Velocity Components}', 'Interpreter', 'latex');
        end

        function generateMassPlot(obj)
            figure;
            hold on;
            
            for i = 1:obj.numSamples
                log = obj.logs{i};
                mass = log.getState('mass');
                time = log.time;
                plot(time, mass, 'LineWidth', 2)
            end
            
            grid on;
            xlabel('\textbf{Time (s)}', 'Interpreter', 'latex');
            ylabel('\textbf{Mass (kg)}', 'Interpreter', 'latex');
            title('\textbf{Mass}', 'Interpreter', 'latex');
        end

        function generateThrustPlot(obj)
            figure;
            hold on;
            
            for i = 1:obj.numSamples
                log = obj.logs{i};
                thrust = log.getThrust();
                time = log.time;
                plot(time, thrust, 'LineWidth', 2)
            end
            
            grid on;
            xlabel('\textbf{Time (s)}', 'Interpreter', 'latex');
            ylabel('\textbf{Thrust (N)}', 'Interpreter', 'latex');
            title('\textbf{Thrust}', 'Interpreter', 'latex');
        end

        function generateAeroMomentPlot(obj)
            figure;
            hold on;
            
            for i = 1:obj.numSamples
                log = obj.logs{i};
                
                Mx = log.getAeroMoment('Mx');  
                My = log.getAeroMoment('My');
                Mz = log.getAeroMoment('Mz');
                
                time = log.time(1:log.currentIdx);
                plot(time, Mx, 'LineWidth', 2)
                plot(time, My, 'LineWidth', 2)
                plot(time, Mz, 'LineWidth', 2)
            end
            
            legend('Mx','My','Mz')
            ylabel('\textbf{Aerodynamic Moments (Nm)}', 'Interpreter', 'latex')
            xlabel('\textbf{Time (s)}', 'Interpreter', 'latex')
            grid on;
        end
        
        function generateControlMomentPlots(obj)
            figure;
            hold on;
            
            for i = 1:obj.numSamples
                log = obj.logs{i};
                
                Mx = log.getControlMoment('Mx');
                My = log.getControlMoment('My');
                Mz = log.getControlMoment('Mz');
                
                time = log.time(1:log.currentIdx);
                plot(time, Mx, 'LineWidth', 2)
                plot(time, My, 'LineWidth', 2)
                plot(time, Mz, 'LineWidth', 2)
            end
            
            legend('Mx','My','Mz')
            ylabel('\textbf{Control Moments (Nm)}', 'Interpreter', 'latex')
            xlabel('\textbf{Time (s)}', 'Interpreter', 'latex')
            grid on;
        end

        function generateAltitudeAndXYPlot(obj)
            % Plot altitude, x, and y coordinates as a function of time
            % Creates a figure with 3 subplots

            figure;
            % Subplot 1: Altitude vs Time
            hold on;

            for i = 1:obj.numSamples
                log = obj.logs{i};
                z_pos = log.getState('z');
                time = log.time(1:log.currentIdx);
                plot(time, z_pos, 'LineWidth', 2);
            end

            grid on;
            xlabel('\textbf{Time (s)}', 'Interpreter', 'latex');
            ylabel('\textbf{Altitude z (m)}', 'Interpreter', 'latex');
            title('\textbf{Altitude vs Time}', 'Interpreter', 'latex');
            hold off;
            
            figure;
            % Subplot 2: X Coordinate vs Time
            subplot(2, 1, 1);
            hold on;

            for i = 1:obj.numSamples
                log = obj.logs{i};
                x_pos = log.getState('x');
                time = log.time(1:log.currentIdx);
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
                plot(time, y_pos, 'LineWidth', 2);
            end

            grid on;
            xlabel('\textbf{Time (s)}', 'Interpreter', 'latex');
            ylabel('\textbf{Y Position (m)}', 'Interpreter', 'latex');
            title('\textbf{Y Coordinate vs Time}', 'Interpreter', 'latex');
            hold off;
        end

        function generateAllPlots(obj)
            obj.generateTrajectoryPlot();
            obj.generateVelocityPlot();
            obj.generateEulerAnglePlot();
            obj.generateAngularVelocityPlot();
            obj.generateMassPlot();
            obj.generateThrustPlot();
            obj.generateAeroMomentPlot();
            obj.generateControlMomentPlots();
            obj.generateAltitudeAndXYPlot();
        end
        
        % Additional helper methods
        function time = getCommonTime(obj)
            % Returns time vector from first log (assumes all synchronized)
            if obj.numSamples > 0
                time = obj.logs{1}.time(1:obj.logs{1}.currentIdx);
            else
                time = [];
            end
        end
        
        function stats = computeStatistics(obj, stateName)
            % Compute mean and std dev across all samples for a given state
            nPoints = obj.logs{1}.currentIdx;
            data = zeros(nPoints, obj.numSamples);
            
            for i = 1:obj.numSamples
                data(:, i) = obj.logs{i}.getState(stateName);
            end
            
            stats.mean = mean(data, 2);
            stats.std = std(data, 0, 2);
            stats.min = min(data, [], 2);
            stats.max = max(data, [], 2);
        end
        
        function plotStateStatistics(obj, stateName)
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
    end
end