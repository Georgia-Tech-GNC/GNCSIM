classdef Analysis < handle
    
    properties
        numSamples;
        time;

        posFlat;
        vel;
        q_bf;
        angVel;
        angleOfAttack;
        sideslipAngle;
        CG;
        angAccel;
        MA_b;
        MT_b;
        ME_b;
        l_p;
        n_a;
        n_r;
        accelFlat;

        Mb;
        Fb;
    end

    methods
        function obj = Analysis(t, x, n, Fb, Mb)
            obj.numSamples = n;
            obj.time = t;
            nTimeSteps = length(obj.time);

            %Need to pre-allocate arrays here
            %Format: (timestep, vector, run number)
            obj.posFlat       = zeros(nTimeSteps, 3, obj.numSamples);
            obj.vel           = zeros(nTimeSteps, 3, obj.numSamples);
            obj.q_bf          = zeros(nTimeSteps, 4, obj.numSamples);
            obj.angVel        = zeros(nTimeSteps, 3, obj.numSamples);
            obj.angleOfAttack = zeros(nTimeSteps, 1, obj.numSamples);
            obj.sideslipAngle = zeros(nTimeSteps, 1, obj.numSamples);
            obj.CG            = zeros(nTimeSteps, 1, obj.numSamples);
            obj.angAccel      = zeros(nTimeSteps, 3, obj.numSamples);
            obj.MA_b          = zeros(nTimeSteps, 3, obj.numSamples);
            obj.MT_b          = zeros(nTimeSteps, 3, obj.numSamples);
            obj.ME_b          = zeros(nTimeSteps, 3, obj.numSamples);
            obj.l_p           = zeros(nTimeSteps, 1, obj.numSamples);
            obj.n_a           = zeros(nTimeSteps, 1, obj.numSamples);
            obj.n_r           = zeros(nTimeSteps, 1, obj.numSamples);
            obj.accelFlat     = zeros(nTimeSteps, 3, obj.numSamples);

            obj.Fb = zeros(nTimeSteps, 3, obj.numSamples);
            obj.Mb = zeros(nTimeSteps, 3, obj.numSamples);

            for i = 1:obj.numSamples
                obj.posFlat(:,:,i)       = transpose(x(2:4,:)); %Will need adaptation for multiple sample
                obj.vel(:,:,i)       = transpose(x(5:7,:));
                obj.q_bf(:,:,i)          = transpose(x(8:11,:));
                obj.angVel(:,:,i)        = transpose(x(12:14,:));
                %obj.angleOfAttack(:,:,i) = 
                %obj.sideslipAngle(:,:,i) = 
                %obj.CG(:,:,i)            = 
                %obj.angAccel(:,:,i)      = 
                %obj.MA_b(:,:,i)          = 
                %obj.MT_b(:,:,i)          = 
                %obj.ME_b(:,:,i)          = 
                %obj.l_p(:,:,i)           = 
                %obj.n_a(:,:,i)           = 
                %obj.n_r(:,:,i)           = 
                %obj.accelFlat(:,:,i)     = 
                obj.Fb(:,:,i) = transpose(Fb);
                obj.Mb(:,:,i) = transpose(Mb);
            end
        end

        function generatePdfFile(obj)
            %Not needed here yet (complete in 34-branch)
        end

        function generateTrajectoryPlot(obj)
            fileName = "traj.png";
            pic = figure;

            plot3(0,0,0,'o','MarkerSize',10)
            hold on
            for i = 1:obj.numSamples
                plot3(obj.posFlat(end,2,i), obj.posFlat(end,3,i), obj.posFlat(end,1,i), 'x','MarkerSize', 10)
                plot3(obj.posFlat(:,2,i), obj.posFlat(:,3,i), obj.posFlat(:,1,i))
            end
            grid on
            % title('\textbf{Closed Loop Trajectories for 1000 MonteCarlo Samples}', 'Interpreter', 'latex')
            xlabel('\textbf{Flat Earth Y (m)}', 'Interpreter', 'latex')
            ylabel('\textbf{Flat Earth Z (m)}', 'Interpreter', 'latex')
            zlabel('\textbf{Flat Earth X (m)}', 'Interpreter', 'latex')
            
            %Uncomment below for PDF generation
            % saveas(pic, fileName);
            % obj.trajPlot = fileName;
        end

        function generateEulerAnglePlot(obj)
            fileName = "yaw.png";
            pic = figure;

            for i = 1:obj.numSamples
                eul = quat2eul(obj.q_bf(:,:,i));
                plot(obj.time,rad2deg(eul(:,:)))
                hold on               
            end
            %title('\textbf{Yaw Angle for 1000 Monte Carlo Samples}', 'Interpreter', 'latex')
            legend('Yaw','Pitch','Roll')
            ylabel('\textbf{Yaw Angle (deg)}', 'Interpreter', 'latex')
            xlabel('\textbf{Time (s)}', 'Interpreter', 'latex')
            grid on
            ax = gca;
            ax.XMinorTick = 'on';
            ax.YMinorTick = 'on';
            ax.MinorGridLineStyle = ':';
            ax.XMinorGrid = 'on';
            ax.YMinorGrid = 'on';
            xlim([0,12]);
            
            %Uncomment below for PDF generation
            % saveas(pic, fileName);
            % obj.eulPlot = fileName;
        end

        function generateMachNumberPlot(obj)
            fileName = "MachNumberBoxPlot.png";

            %Make Plot
            % Plot Mach numbers
            % Calculate Mach Numbers
            a = 335; % m/s (Speed of sound PLACEHOLdER)
            MachMax = zeros(obj.numSamples,1);
            for i = 1:obj.numSamples
                MachNumbers = sqrt(sum(obj.vel(:,:,i).^2, 2))./a;
                MachMax(i) = max(MachNumbers);
            end

            AverageMaxMachNumber = mean(MachMax);
            disp(['Average Maximum Mach Number from all flights: ', num2str(AverageMaxMachNumber)]);
            
            % Plot Altitudes
            %   Calculate Altitudes
            AltMax = zeros(obj.numSamples,1);
            for i = 1:obj.numSamples
                position = obj.posFlat(:,:,i);
                AltMax(i) = max(position(:,1)); %x-position is height
            end

            AverageMaxAltitude = mean(AltMax);
            disp(['Average maximum altitude from all flights: ', num2str(AverageMaxAltitude),'m']);

            %BoxPlots
            % pic = figure;
            % 
            % subplot(1,2,1);
            % boxplot(MachMax);
            % ylabel('Max Mach Number');
            % title('Boxplot of Max Mach');
            % 
            % subplot(1,2,2);
            % boxplot(AltMax);
            % ylabel('Max Altitude (m)');
            % title('Boxplot of Max Altitude');


            % saveas(pic, fileName);
            % obj.MachPlot = fileName;
        end
        
        function generateAeroMomentPlots(obj)
            figure;

            for i = 1:obj.numSamples
                plot(obj.time,obj.Fb)
                hold on               
            end
            legend('Fx','Fy','Fz')
            ylabel('\textbf{Aerodynamic Forces (N)}', 'Interpreter', 'latex')
            xlabel('\textbf{Time (s)}', 'Interpreter', 'latex')

            figure;

            for i = 1:obj.numSamples
                plot(obj.time,obj.Mb)
                hold on               
            end
            legend('Mx','My','Mz')
            ylabel('\textbf{Aerodynamic Moments (Nm)}', 'Interpreter', 'latex')
            xlabel('\textbf{Time (s)}', 'Interpreter', 'latex')

        end

        function GenerateVelocityPlots(obj)
            figure;

            for i = 1:obj.numSamples
                plot(obj.time,obj.vel)
                hold on    
            end
            legend('v_x','v_y','v_z') %this will break for multiple samples, change to 3 plots
            ylabel('\textbf{Velocity in <> Frame (m/s)}', 'Interpreter', 'latex') %check if body or inertial frame
            xlabel('\textbf{Time (s)}', 'Interpreter', 'latex')
        end

        function GenerateAllPlots(obj)
            obj.generateTrajectoryPlot();
            obj.generateMachNumberPlot();
            obj.generateEulerAnglePlot();
            obj.generateAeroMomentPlots();
            obj.GenerateVelocityPlots();
            
        end
    end
end