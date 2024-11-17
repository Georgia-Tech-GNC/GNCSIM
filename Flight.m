classdef Flight < handle
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here

    properties (SetAccess = private)
        tStep; % seconds (double)
        t; % seconds (double)
        runClosedLoop; % boolean
        calcXDots; % boolean
        runLinearModel; % boolean
        applyWind; % boolean
        %useLatinHyperCubeSampling; % boolean
        numStates = 13; 
        numInputs = 3;
        localSimTime; % seconds (double)
        simTime; % seconds (double)
        tSpan; % Vector of doubles
        gravity = 9.81 % m/(s^2)
        uSpan; % length of t span by 3 array of double
        %MCxlsx; % filename of Excel sheet with Monte Carlo information
        windParam; % is set based on Monte Carlo data
        options = odeset('Events',@rocket_events);
        initState;
    end

    methods
        function obj = Flight()
            %UNTITLED2 Construct an instance of this class
            %   Detailed explanation goes here
        end

        function obj= set_tStep(obj, t_step)
            if ~isnumeric(t_step) || t_step < 0
                error('Time step must be a positive double.')
            else
                obj.tStep = t_step;
            end
        end

        function obj= set_runClosedLoop(obj, isClosedLoop)
            if ~islogical(isClosedLoop)
                error('runClosedLoop must be a logical value.')
            else
                obj.runClosedLoop = isClosedLoop;
            end
        end

        function obj= set_calcXDots(obj, calc_xdots)
            if ~islogical(calc_xdots)
                error('calcXdots must be a logical value.')
            else
                obj.calcXDots = calc_xdots;
            end
        end

        function obj= set_runLinearModel(obj, run_linmodel)
            if ~islogical(run_linmodel)
                error('runLinearModel must be a logical value.')
            else
                obj.runLinearModel = run_linmodel;
            end
        end

        function obj= set_applyWind(obj, apply_wind)
            if ~islogical(apply_wind)
                error('applyWind must be a logical value.')
            else
                obj.applyWind = apply_wind;
            end
        end

        % Should be refactored
        % function obj= set_useLatinHyperCubeSampling(obj, latin_sample)
        %     if ~islogical(latin_sample)
        %         error('useLatinHyperCubeSampling must be a logical value.')
        %     else
        %         obj.useLatinHyperCubeSampling = latin_sample;
        %     end
        % end

        function obj= set_numStates(obj, num_states)
            if ~isnumeric(num_states) || num_states < 1
                error('numStates must be a positive double greater than 1.')
            else
                obj.numStates = floor(num_states);
            end
        end

        function obj= set_numInputs(obj, num_in)
            if ~isnumeric(num_in) || num_in < 1
                error('numInputs must be a positive double greater than 1.')
            else
                obj.numInputs = floor(num_in);
            end
        end

        function obj= set_localSimTime(obj, lsimt)
            if ~isnumeric(lsimt) || lsimt < 0
                error('localSimTime must be a positive double.')
            else
                obj.localSimTime = lsimt;
            end
        end

        function obj= set_simTime(obj, simt)
            if ~isnumeric(simt) || simt < 0
                error('simTime must be a positive double.')
            else
                obj.simTime = simt;
            end
        end

        function obj= set_gravity(obj, g)
            if ~isnumeric(g) || g < 0
                error('gravity must be a positive double.')
            else
                obj.gravity = g;
            end
        end

        function obj= set_initState(obj, x0)
            if ~isnumeric(x0) || ~any(x0 ~= obj.numStates, "all")
                error('initState must be a vector of doubles of length numStates.')
            else
                obj.initState = x0;
            end
        end

        function obj= set_tSpan(obj)
            obj.tSpan = 0:obj.tStep:obj.simTime;
        end

        function obj= set_uSpan(obj, u_span)
            [r, c] = size(u_span);
            if ~isnumeric(u_span) || r ~= length(obj.tSpan) || c ~= 3
                error('uSpan must be a tSpan x 3 array of values representing arbitrary external moments on the rocket');
            else
                obj.uSpan = u_span;
            end
        end

        % Should be refactored
        % function obj= set_MCxlsx(obj, fn)
        %     if ~ischar(fn)
        %         error('MCxlsx must be a filename in the form of a char array')
        %     else
        %         obj.MCxlsx = fn;
        %     end
        % end

        function obj= set_windParam(obj, wp)
            obj.windParam = wp; % No conditions check because this comes from Monte Carlo
        end

        function tstep= get_tStep(obj)
            tstep = obj.tStep;
        end

        function rcl= get_runClosedLoop(obj)
            rcl = obj.runClosedLoop;
        end

        function cxd = get_calcXdots(obj)
            cxd = obj.calcXDots;
        end

        function rlm = get_runLinearModel(obj)
            rlm = obj.runLinearModel;
        end

        function aw = get_applyWind(obj)
            aw = obj.applyWind;
        end

        % Should be refactored
        % function ulhcs = get_useLatinHyperCubeSampling(obj)
        %     ulhcs = obj.useLatinHyperCubeSampling;
        % end

        function ns = get_numStates(obj)
            ns = obj.numStates;
        end
        
        function ni = get_numInputs(obj)
            ni = obj.numInputs;
        end

        function lst = get_localSimTime(obj)
            lst = obj.localSimTime;
        end

        function st = get_simTime(obj)
            st = obj.simTime;
        end

        function ts = get_tSpan(obj)
            ts = obj.tSpan;
        end

        function g = get_gravity(obj)
            g = obj.gravity;
        end

        function us = get_uSpan(obj)
            us = obj.uSpan;
        end

        function is = get_initState(obj)
            is = obj.initState;
        end
    
        % Should be refactored
        % function mcexcel = get_MCxlsx(obj)
        %     mcexcel = obj.MCxlsx;
        % end

        function opts = get_options(obj)
            opts = obj.options;
        end

        % This function should be rolled into the Monte Carlo class and
        % refactored
        % function mcParams = getRandomSample(xlsxFile,runIndex)
        %     % getRandomSample
        %     xlsxFile = readcell(xlsxFile);
        % 
        %     gustAmplitude = random(makedist(xlsxFile{2*runIndex,4},'mu',xlsxFile{2*runIndex,5},'sigma',xlsxFile{2*runIndex,6}/3));
        %     gustLength = random(makedist(xlsxFile{1+2*runIndex,4},'mu',xlsxFile{1+2*runIndex,5},'sigma',xlsxFile{1+2*runIndex,6}/3));
        %     windParameter = [gustAmplitude gustLength];
        % 
        %     % INSERT NEW MONTE CARLO PARAM (READ FROM EXCEL SHEET) HERE
        %     wetMass = 38.87;% update to read from excel sheet
        %     dryMass = 30.96; % update to read from excel sheet
        %     peakThrust = 2262; % update to read from excel sheet
        %     burnTime = 13.1; % update to read from excel sheet
        %     % thrustCurve = generate_thrust_curve(peakThrust, burnTime);
        %     wetRollMOI = 0.2109; % update to read from excel sheet
        %     wetPitchMOI = 21.1212; % update to read from excel sheet
        %     wetYawMOI = 21.1202; % update to read from excel sheet
        %     dryRollMOI = 0.2036; % update to read from excel sheet
        %     dryPitchMOI = 18.99; % update to read from excel sheet
        %     dryYawMOI = 18.988; % update to read from excel sheet
        % 
        %     % UPDATE OUTPUT STRUCT
        %     mcParams.windParam = windParameter;
        %     mcParams.wetMass = wetMass;
        %     mcParams.dryMass = dryMass;
        %     mcParams.peakThrust = peakThrust;
        %     mcParams.burnTime = burnTime;
        %     mcParams.thrustCurve = thrustCurve;
        %     mcParams.wetMOI = [wetRollMOI 0 0;
        %                         0 wetPitchMOI 0;
        %                         0 0 wetYawMOI];
        %     mcParams.dryMOI = [dryRollMOI 0 0;
        %                         0 dryPitchMOI 0;
        %                         0 0 dryYawMOI];
        % end

        function log = runFlight(obj, Rocket, Environment, Aerodynamics, TrimPointObj)
            tic
            log = struct();
            fprintf('Running flight...\n')

            % Number of LQR intervals to simulate
            % currently each LQR interval is 1 second
            % NOTE: FSW will be switching LQR gains at a much faster rate (10 Hz)
            numIntervals = obj.simTime / obj.localSimTime;
            % Allocate log memory for current MC run
            tempMClog(numIntervals) = struct();

            if obj.get_runLinearModel()
                tempMClinLog(numIntervals) = struct();
            end
                
            % The below will be rolled into the Monte Carlo class
            % if obj.get_useLatinHyperCubeSampling()
            %     mcParams = getLatinHyperCubeSample(obj.get_MCxlsx(), runIndex);
            % else
            %     mcParams = getRandomSample(obj.get_MCxlsx(), runIndex);
            % end

            for lqrIndex =  1:numIntervals
                fprintf('Current LQR config: %d \n', lqrIndex);

                % Get initial state for each LQR time interval
                if lqrIndex == 1
                    initial_state = Flight.get_initState();
                    lin_initial_state = initial_state([4 5 6 11 12 13 8 9 10]);
                else
                    initial_state =prevFinalState;
                    lin_initial_state = initial_state([4 5 6 11 12 13 8 9 10]);
                end

                % Solve linear and nonlinear systems for localSimTime
                % seconds
                local_startTime = (lqrIndex-1) * obj.get_localSimTime();
                local_endTime = local_startTime + obj.get_localSimTime();
                local_tSpan = local_startTime:obj.get_tStep():local_endTime;

                % Solve nonlinear trajectory for the current time
                % interval
                ode_options = obj.get_options();
                [traj.t, traj.x, traj.te, traj.ye, traj.ie] = ode45(@(t,x) rocketModel(t, x, Rocket, Environment, obj, Aerodynamics, TrimPointObj), ...
                                         local_tSpan, ...
                                         initial_state, ...
                                         ode_options);
                prevFinalState = traj.x(end, :)';
             
                % Log nonlinear traj data for current LQR interval to
                % the log for the current run
                tempMClog(lqrIndex).time = traj.t;
                tempMClog(lqrIndex).posFlat = traj.x(:, 1:3);
                tempMClog(lqrIndex).velBody = traj.x(:, 4:6);
                tempMClog(lqrIndex).q_bf = traj.x(:, 7:10);
                tempMClog(lqrIndex).angVel = traj.x(:, 11:13);

                if config.calcXdots == 1
                    tempMClog(lqrIndex).angleOfAttack = zeros([length(traj.t),1]); % alpha
                    tempMClog(lqrIndex).sideslipAngle = zeros([length(traj.t),1]); % beta
                    tempMClog(lqrIndex).CG = zeros([length(traj.t)],1);
                    tempMClog(lqrIndex).angAccel = zeros(length(traj.t),3);
                    tempMClog(lqrIndex).MA_b = zeros(length(traj.t),3);
                    tempMClog(lqrIndex).MT_b = zeros(length(traj.t),3);
                    tempMClog(lqrIndex).ME_b = zeros(length(traj.t),3);
                    tempMClog(lqrIndex).l_p = zeros(length(traj.t),1);
                    tempMClog(lqrIndex).n_a = zeros(length(traj.t),1);
                    tempMClog(lqrIndex).n_r = zeros(length(traj.t),1);
                    for i = 1:length(traj.t)
                        [xdot,modelLog] = rocketModel(traj.t(i),traj.x(i,:)',Rocket,Environment,obj,Aerodynamics,TrimPointObj);
                        tempMClog(lqrIndex).angleOfAttack(i) = modelLog.alpha;
                        tempMClog(lqrIndex).sideslipAngle(i) = modelLog.beta;
                        tempMClog(lqrIndex).CG(i) = modelLog.CG;
                        tempMClog(lqrIndex).angAccel(i,:) = xdot(11:13);
                        tempMClog(lqrIndex).MA_b(i,:) = modelLog.MA_b;
                        tempMClog(lqrIndex).MT_b(i,:) = modelLog.MT_b;
                        tempMClog(lqrIndex).ME_b(i,:) = modelLog.ME_b;
                        tempMClog(lqrIndex).l_p(i,:) = modelLog.l_p;
                        tempMClog(lqrIndex).n_a(i,:) = modelLog.n_a;
                        tempMClog(lqrIndex).n_r(i,:) = modelLog.n_r;  
                    end
                end

                if obj.get_runLinearModel()
                % Solve linear trajectory for current time interval
                    [linTraj.t, linTraj.x] = ode45(@(t,x) linearRocketModelrocketModel(t, x, Rocket, Environment, obj, Aerodynamics, TrimPointObj),...
                                                local_tSpan,...
                                                lin_initial_state);
                    tempMClinLog(lqrIndex).time = linTraj.t;
                    tempMClinLog(lqrIndex).velBody = linTraj.x(:,1:3);
                    tempMClinLog(lqrIndex).q_bf = [ones(length(tempMClinLog(lqrIndex).time),1) linTraj.x(:,7:9)];
                    tempMClinLog(lqrIndex).angVel = linTraj.x(:,4:6);

                    if obj.get_calcXdots == 1
                        tempMClinLog(lqrIndex).v_f = zeros(length(tempMClinLog(lqrIndex).time),3);
                        tempMClinLog(lqrIndex).rollMoment = zeros(length(tempMClinLog(lqrIndex).time));
                        tempMClinLog(lqrIndex).pitchMoment = zeros(length(tempMClinLog(lqrIndex).time));
                        tempMClinLog(lqrIndex).yawMoment = zeros(length(tempMClinLog(lqrIndex).time));
                        tempMClinLog(lqrIndex).thrust = zeros(length(tempMClinLog(lqrIndex).time));
    
                        for i = 1:length(linTraj.t)
                            [xdot,modelLog] = linearRocketModel(linTraj.t(i),linTraj.x(i,:)',linConfig); % TODO: Change for linear model
                            tempMClinLog(lqrIndex).v_f(i,:) = modelLog.v_f;
                            tempMClinLog(lqrIndex).rollMoment(i) = modelLog.rollMoment;
                            tempMClinLog(lqrIndex).pitchMoment(i) = modelLog.pitchMoment;
                            tempMClinLog(lqrIndex).yawMoment(i) = modelLog.yawMoment;
                            tempMClinLog(lqrIndex).thrust(i) = modelLog.thrust;
    
                        end
                    end
                end
            end
            
                % Integrate data from current MC log
                tempLoggedData.time = vertcat(tempMClog.time);
                tempLoggedData.posFlat = vertcat(tempMClog.posFlat);
                tempLoggedData.velBody = vertcat(tempMClog.velBody);
                tempLoggedData.q_bf = vertcat(tempMClog.q_bf);
                tempLoggedData.angVel = vertcat(tempMClog.angVel);
        
                if config.calcXdots == 1
                    tempLoggedData.angleOfAttack = vertcat(tempMClog.angleOfAttack);
                    tempLoggedData.sideslipAngle = vertcat(tempMClog.sideslipAngle);
                    tempLoggedData.CG = vertcat(tempMClog.CG);
                    tempLoggedData.angAccel = vertcat(tempMClog.angAccel);
                    tempLoggedData.MA_b = vertcat(tempMClog.MA_b);
                    tempLoggedData.MT_b = vertcat(tempMClog.MT_b);
                    tempLoggedData.ME_b = vertcat(tempMClog.ME_b);
                    tempLoggedData.l_p = vertcat(tempMClog.l_p);
                    tempLoggedData.n_a = vertcat(tempMClog.n_a);
                    tempLoggedData.n_r = vertcat(tempMClog.n_r);
                    
                end
        
                if testConfig.runLinearModel
                    tempLinLoggedData.time = vertcat(tempMClinLog.time);
                    tempLinLoggedData.velBody = vertcat(tempMClinLog.velBody);
                    tempLinLoggedData.q_bf = vertcat(tempMClinLog.q_bf);
                    tempLinLoggedData.angVel = vertcat(tempMClinLog.angVel);
        
                    if config.calcXdots == 1
                        tempLinLoggedData.v_f = vertcat(tempMClinLog.v_f);
                        tempLinLoggedData.rollMoment = vertcat(tempMClinLog.rollMoment);
                        tempLinLoggedData.pitchMoment = vertcat(tempMClinLog.pitchMoment);
                        tempLinLoggedData.yawMoment = vertcat(tempMClinLog.yawMoment);
                        tempLinLoggedData.thrust = vertcat(tempMClinLog.thrust);
                    end
                end
                log.loggedData = tempLoggedData;
                if testConfig.runLinearModel
                    log.linLoggedData = tempLinLoggedData;
                end
        
                % End of current run
                clear tempMClog % this may be horrible practice
                fprintf('--------------------------------------\n')

            fprintf("Finished run!\n");
            toc
        end

        function []= plot_sim_dashboard(obj, log, Rocket, Environment)

            % Plot a flight in a dashboard
            figure;
            
            % 3D Trajectory Plot %
            subplot(3,4,1)
            plot3(0,0,0,'o','MarkerSize',10)
            hold on
            plot3(log.loggedData.posFlat(end,2),log.loggedData.posFlat(end,3),log.loggedData.posFlat(end,1),'x','MarkerSize',10)
            hold on
            plot3(log.loggedData.posFlat(:,2),log.loggedData.posFlat(:,3),log.loggedData.posFlat(:,1))
            grid on
            xlabel('x')
            ylabel('y')
            zlabel('z')
            if obj.get_runClosedLoop()
                title('Closed Loop Trajectory')
            else
                title('Open Loop Trajectory')
            end
            % % % % % % % % % % % %
            % Body Velocity %
            subplot(3,4,2)
            plot(log.loggedData.time,log.loggedData.velBody)
            legend('x','y','z')
            title('Body Velocity')
            ylabel('v_b (m/s)')
            xlabel('Time (s)')
            grid on

            % % % % % % % % % % % %
            % Body Angular Velocity %
            subplot(3,4,3)
            plot(log.loggedData.time,rad2deg(log.loggedData.angVel), ...
                    'LineWidth', 1.5);
            hold on
            title('Body Angular Velocity')
            ylabel('wbi_b (deg/s)','FontSize', 12)
            xlabel('time (s)','FontSize', 12)
            legend('Roll','Pitch','Yaw')
            grid on
            hold off
            
            % Enable minor tick marks
            ax = gca; % Get current axis
            ax.XMinorTick = 'on'; % Turn on minor ticks for x-axis
            ax.YMinorTick = 'on'; % Turn on minor ticks for y-axis
            
            % Customize the spacing of minor ticks
            ax.MinorGridLineStyle = ':'; % Optional: dotted grid for minor ticks
            ax.XMinorGrid = 'on'; % Turn on minor grid for x-axis
            ax.YMinorGrid = 'on'; % Turn on minor grid for y-axis

            % % % % % % % % % % % %
            % Quaternion %
            subplot(3,4,4)
            plot(log.loggedData.time,log.loggedData.q_bf)
            legend('q0','q1','q2','q3')
            % % % % % % % % % % % %
            % Monte Carlo Parameters %

            % TODO
            % Create a text display for Flight, Rocket, and Environment
            % Parameters that are varied by the Monte Carlo analysis

            % % % % % % % % % % % %
            % Angle of Attack %
            subplot(3,4,6)
            plot(log.loggedData.time,rad2deg(log.loggedData.angleOfAttack), ...
                    'LineWidth', 1.5);
            hold on
            title('Angle of Attack Trajectory')
            xlabel('Time (s)','FontSize', 12)
            ylabel('Alpha (deg)','FontSize', 12)
            % legend('1 deg','3 deg','6 deg', '9 deg')
            grid on
            hold off
            
            % Enable minor tick marks
            ax = gca; % Get current axis
            ax.XMinorTick = 'on'; % Turn on minor ticks for x-axis
            ax.YMinorTick = 'on'; % Turn on minor ticks for y-axis
            
            % Customize the spacing of minor ticks
            ax.MinorGridLineStyle = ':'; % Optional: dotted grid for minor ticks
            ax.XMinorGrid = 'on'; % Turn on minor grid for x-axis
            ax.YMinorGrid = 'on'; % Turn on minor grid for y-axis
            % % % % % % % % % % % %
            % Angular Acceleration %
            subplot(3,4,7)
            plot(log.loggedData.time,rad2deg(log.loggedData.angAccel))
            legend('x','y','z')
            % % % % % % % % % % % %
            % Euler Angles %
            eul = quat2eul(log.loggedData.q_bf);
            plot(log.loggedData.time,rad2deg(eul(:,3:-1:1)))
            legend('x','y','z', 'Location', 'best')
            title('Euler Angles')
            ylabel('Angle (deg)')
            xlabel('Time (s)')
            grid on
            title("Euler Angles over Time")
            ax = gca;
            ax.XMinorTick = 'on';
            ax.YMinorTick = 'on';
            ax.MinorGridLineStyle = ':';
            ax.XMinorGrid = 'on';
            ax.YMinorGrid = 'on';
            % % % % % % % % % % % %
            % Linearization Body Velocity Error %

            if obj.get_runLinearModel()
                subplot(3,4,9)
                    plot(log.loggedData.time,log.loggedData.velBody(:,1), ...
                        'LineWidth', 1.5);
                    hold on
                    plot(log.linLoggedData.time,log.linLoggedData.velBody(:,1), ...
                            'LineWidth', 1.5);
                    legend('Nonlinear','Linear Model')
                    hold on

                if obj.get_runClosedLoop()
                    title('Closed Loop Velocity, Linearization Comparison')
                else
                    title('Open Loop Velocity, Linearization Comparison')
                end
                ylabel('Velocity (m/s)','FontSize', 12)
                xlabel('Time (s)','FontSize', 12)
                grid on
                hold off
                
                % Enable minor tick marks
                ax = gca; % Get current axis
                ax.XMinorTick = 'on'; % Turn on minor ticks for x-axis
                ax.YMinorTick = 'on'; % Turn on minor ticks for y-axis
                
                % Customize the spacing of minor ticks
                ax.MinorGridLineStyle = ':'; % Optional: dotted grid for minor ticks
                ax.XMinorGrid = 'on'; % Turn on minor grid for x-axis
                ax.YMinorGrid = 'on'; % Turn on minor grid for y-axis
                
            end

            % % % % % % % % % % % %
            % External Input Moments %
            subplot(3,4,9)
            plot(log.loggedData.time,log.loggedData.ME_b, ...
                    'LineWidth', 1.5);
            hold on
            title('External Moment')
            ylabel('ME_b (m/s)','FontSize', 12)
            xlabel('time (s)','FontSize', 12)
            grid on
            hold off
            % % % % % % % % % % % %
            % Controller Response Moments %
            subplot(3,4,11)
            plot(log.loggedData.time, log.loggedData.MT_b)
            legend('Roll', 'Pitch', 'Yaw')
            title('Controller Response Moment')
            ylabel('Controller Moment (Nm)')
            xlabel('Time (s)')
            grid on
            % % % % % % % % % % % %
            % Aerodynamic Moments %
            subplot(3,4,12)

            plot(log.loggedData.time,log.loggedData.l_p, ...
                'LineWidth', 1.5);
            hold on
            plot(log.loggedData.time,log.loggedData.n_a, ...
                'LineWidth', 1.5);
            hold on
            plot(log.loggedData.time,log.loggedData.n_r, ...
                'LineWidth', 1.5);
            hold on     
            title('Aerodynamic Moments')
            ylabel('Moment (Nm)','FontSize', 12)
            xlabel('Time (s)','FontSize', 12)
            grid on
            legend('l_p','n_a','n_r','MA_b')
        end

        function [value, isterminal, direction]= rocket_events(t,x)
            %rocket_events - called by ode45 to determine various rocket
            %events, i.e., rocket hitting ground
            value = x(1); % Define event as px = 0
            isterminal=1; %stop integration when event happens
            direction = -1; %only stop is px is increasing (altitude is decreasing)
        end

    end
end
