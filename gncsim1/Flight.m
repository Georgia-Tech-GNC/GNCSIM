% Main Authors: Patrick Barry, Albert Zheng

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
        numStates = 13; 
        numInputs = 3;
        localSimTime; % seconds (double)
        simTime; % seconds (double)
        tSpan; % Vector of doubles
        gravity = 9.81 % m/(s^2)
        uSpan; % length of t span by 3 array of double
        windParam; % is set based on Monte Carlo data
        odeOptions; % odeevents options
        initState;
        controlsTime;
        odeSolverStartTime;
        odeSolverMaxTime = 1000; % set to 1000s by default to avoid errors
        odeSolverTerminationFlag = false;
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

        function obj= set_windParam(obj, wp)
            obj.windParam = wp; % No conditions check because this comes from Monte Carlo
        end
        
        function obj = set_odeSolverMaxTime(obj, value)
            validateattributes(value, {'numeric'}, {'nonnegative', 'scalar'}, mfilename, 'odeSolverMaxTime');
            obj.odeSolverMaxTime = value;
        end

        function obj = set_controlsTime(obj,newTime)
            obj.controlsTime = newTime;
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

        function opts = get_options(obj)
            opts = obj.options;
        end

        function controlsTime = get_controlsTime(obj)
            controlsTime = obj.controlsTime;
        end

        function log = runFlight(obj, Rocket, Environment, Aerodynamics, ControlObj)
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

            for lqrIndex =  1:numIntervals
                if obj.get_runLinearModel()
                    fprintf('Current linear config: %d \n', lqrIndex);
                end

                % Get initial state for each LQR time interval
                if lqrIndex == 1
                    initial_state = obj.get_initState();
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
                obj.odeSolverStartTime = tic;
                obj.odeSolverTerminationFlag = false;
                obj.odeOptions = odeset('Events', @Flight.rocket_events,'OutputFcn', @(t, x, flag) obj.stopSolver(t, x, flag, obj.odeSolverStartTime, obj.odeSolverMaxTime));
                
                % [traj.t, traj.x, traj.te, traj.ye, traj.ie] = ode45(@(t,x) rocketModel(t, x, Rocket, Environment, obj, Aerodynamics, TrimPointObj), ...
                %                         local_tSpan, ...
                %                         initial_state, ...
                %                         ode_options);

                % Euler Propagation
                %{
                    traj.t = [];
                    traj.x = [];
                    x = initial_state;
                    for time = local_tSpan
                        traj.t = [traj.t; time];
                        traj.x = [traj.x; x'];
                        [xDot, ~] = rocketModel(time, x, Rocket, Environment, obj, Aerodynamics, ControlObj);
                        x = x + xDot*obj.get_tStep();
    
                        discrete_second_check = (time == fix(time)); % Check if simulation time is an integer
                        if discrete_second_check
                            curr_time_str = num2str(time);
                            printed_str = [curr_time_str ' seconds of flight simulated.'];
                            disp(printed_str)
                        end
                    end
                %}
                % End Euler Propagation
                
                %[traj.t, traj.x] = ode45(@(t,x) rocketModel(t, x, Rocket, Environment, obj, Aerodynamics, ControlObj), local_tSpan, initial_state, obj.odeOptions);
                [traj.t, traj.x] = obj.timestepper(initial_state, obj.get_tStep(), local_tSpan, Rocket, Environment, Aerodynamics, ControlObj);

                prevFinalState = traj.x(end, :)';
             
                % Log nonlinear traj data for current LQR interval to
                % the log for the current run
                tempMClog(lqrIndex).time = traj.t;
                tempMClog(lqrIndex).posFlat = traj.x(:, 1:3);
                tempMClog(lqrIndex).velBody = traj.x(:, 4:6);
                tempMClog(lqrIndex).q_bf = traj.x(:, 7:10);
                tempMClog(lqrIndex).angVel = traj.x(:, 11:13);

                if obj.get_calcXdots()
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
                        [xdot,modelLog] = rocketModel(traj.t(i),traj.x(i,:)',Rocket,Environment,obj,Aerodynamics,ControlObj);
                        tempMClog(lqrIndex).angleOfAttack(i) = modelLog.alpha;
                        tempMClog(lqrIndex).sideslipAngle(i) = modelLog.beta;
                        tempMClog(lqrIndex).CG(i) = modelLog.CG;
                        tempMClog(lqrIndex).angAccel(i,:) = xdot(11:13);
                        tempMClog(lqrIndex).MA_b(i,:) = modelLog.MA_b;
                        tempMClog(lqrIndex).MT_b(i,:) = modelLog.MT_b;
                        tempMClog(lqrIndex).ME_b(i,:) = modelLog.ME_b;
                        tempMClog(lqrIndex).accelFlat(i,:) = modelLog.accelFlat;
                        %tempMClog(lqrIndex).l_p(i,:) = modelLog.l_p; % doesn't exist in rocketModel
                        %tempMClog(lqrIndex).n_a(i,:) = modelLog.n_a; % doesn't exist in rocketModel
                        %tempMClog(lqrIndex).n_r(i,:) = modelLog.n_r; % doesn't exist in rocketModel
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

                    if obj.get_calcXdots()
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
                tempLoggedData.termination_flag = obj.odeSolverTerminationFlag;
        
                if obj.get_calcXdots()
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
                    tempLoggedData.accelFlat = vertcat(tempMClog.accelFlat);
                    
                end
        
                if obj.get_runLinearModel()
                    tempLinLoggedData.time = vertcat(tempMClinLog.time);
                    tempLinLoggedData.velBody = vertcat(tempMClinLog.velBody);
                    tempLinLoggedData.q_bf = vertcat(tempMClinLog.q_bf);
                    tempLinLoggedData.angVel = vertcat(tempMClinLog.angVel);
        
                    if obj.get_calcXdots()
                        tempLinLoggedData.v_f = vertcat(tempMClinLog.v_f);
                        tempLinLoggedData.rollMoment = vertcat(tempMClinLog.rollMoment);
                        tempLinLoggedData.pitchMoment = vertcat(tempMClinLog.pitchMoment);
                        tempLinLoggedData.yawMoment = vertcat(tempMClinLog.yawMoment);
                        tempLinLoggedData.thrust = vertcat(tempMClinLog.thrust);
                    end
                end
                log.loggedData = tempLoggedData;
                if obj.get_runLinearModel()
                    log.linLoggedData = tempLinLoggedData;
                end
        
                % End of current run
                clear tempMClog % this may be horrible practice
                fprintf('--------------------------------------\n')

            fprintf("Finished run!\n");
            toc
        end

        function [] = plot_sim_dashboard(obj, log, Rocket, Environment)
            if obj.runLinearModel
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
                xlabel('Flat Earth Y (m)')
                ylabel('Flat Earth Z (m)')
                zlabel('Flat Earth X (m)')
                if obj.get_runClosedLoop()
                    title('Closed Loop Trajectory')
                else
                    title('Open Loop Trajectory')
                end
                % % % % % % % % % % % %
                % Body Velocity %
                subplot(3,4,2)
                plot(log.loggedData.time,log.loggedData.velBody)
                legend('u','v','w', 'Location', 'northwest')
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
                ylabel('w_b^f (deg/s)','FontSize', 12)
                xlabel('Time (s)','FontSize', 12)
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
                title('Quaternion Plot')
                xlabel('Time (s)','FontSize', 12)
                ylabel('Quaternion Components','FontSize', 12)
                % Enable minor tick marks
                ax = gca; % Get current axis
                ax.XMinorTick = 'on'; % Turn on minor ticks for x-axis
                ax.YMinorTick = 'on'; % Turn on minor ticks for y-axis            
                % Customize the spacing of minor ticks
                ax.MinorGridLineStyle = ':'; % Optional: dotted grid for minor ticks
                ax.XMinorGrid = 'on'; % Turn on minor grid for x-axis
                ax.YMinorGrid = 'on'; % Turn on minor grid for y-axis
                % % % % % % % % % % % %
                % Monte Carlo Parameters %
                subplot(3,4,5)

                yvals = -2:1:10;
                xvals = 0.1*ones(1, length(yvals));
                plot(xvals,yvals, 'LineWidth', 5)
                ylim([-1 10])
                xlim([0 20])
                hold on
                title('Parameters of Interest')
                text(0.5, 0, 'Dry Mass (kg): ');
                text(5.5, 0, num2str(Rocket.get_dryMass()));

                text(0.5, 1, 'Final CG (m): ');
                text(5.5, 1, num2str(Rocket.get_finalCG()));

                tc = Rocket.get_thrustCurve();
                max_thrust = max(tc(:, 2));
                text(0.5, 2, 'Peak Thrust (N): ');
                text(6, 2, num2str(max_thrust));

                dry_moi_matrix = Rocket.get_dryMOI();
                dry_mois = [dry_moi_matrix(1,1), dry_moi_matrix(2,2), dry_moi_matrix(3,3)];
                text(0.5, 3, 'Dry MOIs (kgm^2): ');
                text(6.5, 3, num2str(dry_mois));

                altitudes = Environment.get_altitudes();
                windspeed_on_ground = Environment.get_wind_speed(altitudes(1));
                text(0.5, 4, 'Ground Windspeed (m/s): ');
                text(9, 4, num2str(windspeed_on_ground));

                set(gca,'XTick',[], 'YTick', []);
        
                % % % % % % % % % % % %
                % Angle of Attack %
                subplot(3,4,6)
                plot(log.loggedData.time,rad2deg(log.loggedData.angleOfAttack), ...
                        'LineWidth', 1.5);
                hold on
                title('Angle of Attack Trajectory')
                xlabel('Time (s)','FontSize', 12)
                ylabel('\alpha (deg)','FontSize', 12)
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
                plot(log.loggedData.time,rad2deg(log.loggedData.angAccel(:, 1)), 'LineWidth', 0.5)
                hold on
                plot(log.loggedData.time,rad2deg(log.loggedData.angAccel(:, 2)), 'LineWidth', 0.5)
                plot(log.loggedData.time,rad2deg(log.loggedData.angAccel(:, 3)), 'LineWidth', 0.5)
                legend('Roll','Pitch','Yaw', 'Location', 'best')
                title('Angular Acceleration')
                ylabel('Angular Acceleration (deg/s^2)', 'FontSize', 12)
                xlabel('Time (s)', 'FontSize', 12)
                % Enable minor tick marks
                ax = gca; % Get current axis
                ax.XMinorTick = 'on'; % Turn on minor ticks for x-axis
                ax.YMinorTick = 'on'; % Turn on minor ticks for y-axis
                % Customize the spacing of minor ticks
                ax.MinorGridLineStyle = ':'; % Optional: dotted grid for minor ticks
                ax.XMinorGrid = 'on'; % Turn on minor grid for x-axis
                ax.YMinorGrid = 'on'; % Turn on minor grid for y-axis
                % % % % % % % % % % % %
                % Euler Angles %
                subplot(3,4,8)
                eul = quat2eul(log.loggedData.q_bf);
                plot(log.loggedData.time,rad2deg(eul(:,3:-1:1)))
                legend('Phi (Roll)','Theta (Pitch)','Psi (Yaw)', 'Location', 'best')
                title('Euler Angles')
                ylabel('Angle (deg)', 'FontSize', 12)
                xlabel('Time (s)', 'FontSize', 12)
                grid on
                ax = gca;
                ax.XMinorTick = 'on';
                ax.YMinorTick = 'on';
                ax.MinorGridLineStyle = ':';
                ax.XMinorGrid = 'on';
                ax.YMinorGrid = 'on';
                % % % % % % % % % % % %
                % Linearization Body Velocity Error %
                subplot(3,4,9)
                plot(log.loggedData.time,log.loggedData.velBody(:,1), 'LineWidth', 1.5);
                hold on
                plot(log.linLoggedData.time,log.linLoggedData.velBody(:,1), 'LineWidth', 1.5);
                legend('Nonlinear','Linear Model')
                hold on
                title('Linearized Velocity Comparison')
                ylabel('Body-x Velocity (m/s)','FontSize', 12)
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
        
                % % % % % % % % % % % %
                % External Input Moments %
                subplot(3,4,10)
                plot(log.loggedData.time,log.loggedData.ME_b, 'LineWidth', 1.5);
                hold on
                title('Manual External Moment Inputs')
                ylabel('ME_b (m/s)','FontSize', 12)
                xlabel('Time (s)','FontSize', 12)
                legend('M_x', 'M_y', 'M_z', 'Location', 'northeast')
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
                % Controller Response Moments %
                subplot(3,4,11)
                plot(log.loggedData.time, log.loggedData.MT_b)
                legend('M_{roll}', 'M_{pitch}', 'M_{yaw}', 'Location', 'northeast')
                title('Controller Response Moment')
                ylabel('Controller Moment (Nm)')
                xlabel('Time (s)')
                grid on
                % % % % % % % % % % % %
                % Aerodynamic Moments %
                subplot(3,4,12)
                plot(log.loggedData.time,log.loggedData.MA_b, 'LineWidth', 1.5);   
                title('Aerodynamic Moment')
                ylabel('Moment (Nm)','FontSize', 12)
                xlabel('Time (s)','FontSize', 12)
                grid on
                legend('MA_x','MA_y','MA_z', 'Location', 'northwest')
            else % nonlinear model only
                figure;
                
                % 3D Trajectory Plot %
                subplot(3,4,[1 5])
                plot3(0,0,0,'o','MarkerSize',10)
                hold on
                plot3(log.loggedData.posFlat(end,2),log.loggedData.posFlat(end,3),log.loggedData.posFlat(end,1),'x','MarkerSize',10)
                hold on
                plot3(log.loggedData.posFlat(:,2),log.loggedData.posFlat(:,3),log.loggedData.posFlat(:,1))
                grid on
                xlabel('Flat Earth Y (m)')
                ylabel('Flat Earth Z (m)')
                zlabel('Flat Earth X (m)')
                if obj.get_runClosedLoop()
                    title('Closed Loop Trajectory')
                else
                    title('Open Loop Trajectory')
                end
                % % % % % % % % % % % %
                % Body Velocity %
                subplot(3,4,2)
                plot(log.loggedData.time,log.loggedData.velBody)
                legend('u','v','w', 'Location', 'northwest')
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
                ylabel('w_b^f (deg/s)','FontSize', 12)
                xlabel('Time (s)','FontSize', 12)
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
                title('Quaternion Plot')
                xlabel('Time (s)','FontSize', 12)
                ylabel('Quaternion Components','FontSize', 12)
                % Enable minor tick marks
                ax = gca; % Get current axis
                ax.XMinorTick = 'on'; % Turn on minor ticks for x-axis
                ax.YMinorTick = 'on'; % Turn on minor ticks for y-axis            
                % Customize the spacing of minor ticks
                ax.MinorGridLineStyle = ':'; % Optional: dotted grid for minor ticks
                ax.XMinorGrid = 'on'; % Turn on minor grid for x-axis
                ax.YMinorGrid = 'on'; % Turn on minor grid for y-axis
                % % % % % % % % % % % %
                % Angle of Attack %
                subplot(3,4,6)
                plot(log.loggedData.time,rad2deg(log.loggedData.angleOfAttack), ...
                        'LineWidth', 1.5);
                hold on
                title('Angle of Attack Trajectory')
                xlabel('Time (s)','FontSize', 12)
                ylabel('\alpha (deg)','FontSize', 12)
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
                plot(log.loggedData.time,rad2deg(log.loggedData.angAccel(:, 1)), 'LineWidth', 0.5)
                hold on
                plot(log.loggedData.time,rad2deg(log.loggedData.angAccel(:, 2)), 'LineWidth', 0.5)
                plot(log.loggedData.time,rad2deg(log.loggedData.angAccel(:, 3)), 'LineWidth', 0.5)
                legend('Roll','Pitch','Yaw', 'Location', 'best')
                title('Angular Acceleration')
                ylabel('Angular Acceleration (deg/s^2)', 'FontSize', 12)
                xlabel('Time (s)', 'FontSize', 12)
                % Enable minor tick marks
                ax = gca; % Get current axis
                ax.XMinorTick = 'on'; % Turn on minor ticks for x-axis
                ax.YMinorTick = 'on'; % Turn on minor ticks for y-axis
                % Customize the spacing of minor ticks
                ax.MinorGridLineStyle = ':'; % Optional: dotted grid for minor ticks
                ax.XMinorGrid = 'on'; % Turn on minor grid for x-axis
                ax.YMinorGrid = 'on'; % Turn on minor grid for y-axis
                % % % % % % % % % % % %
                % Euler Angles %
                subplot(3,4,8)
                eul = quat2eul(log.loggedData.q_bf);
                plot(log.loggedData.time,rad2deg(eul(:,3:-1:1)))
                legend('Phi (Roll)','Theta (Pitch)','Psi (Yaw)', 'Location', 'best')
                title('Euler Angles')
                ylabel('Angle (deg)', 'FontSize', 12)
                xlabel('Time (s)', 'FontSize', 12)
                grid on
                ax = gca;
                ax.XMinorTick = 'on';
                ax.YMinorTick = 'on';
                ax.MinorGridLineStyle = ':';
                ax.XMinorGrid = 'on';
                ax.YMinorGrid = 'on';
                % % % % % % % % % % % %
                % Monte Carlo Parameters %
                subplot(3,4,9)
                
                yvals = -2:1:10;
                xvals = 0.1*ones(1, length(yvals));
                plot(xvals,yvals, 'LineWidth', 5)
                ylim([-1 10])
                xlim([0 20])
                hold on
                title('Parameters of Interest')
                text(0.5, 0, 'Dry Mass (kg): ');
                text(5.5, 0, num2str(Rocket.get_dryMass()));

                text(0.5, 1, 'Final CG (m): ');
                text(5.5, 1, num2str(Rocket.get_finalCG()));

                tc = Rocket.get_thrustCurve();
                max_thrust = max(tc(:, 2));
                text(0.5, 2, 'Peak Thrust (N): ');
                text(6, 2, num2str(max_thrust));

                dry_moi_matrix = Rocket.get_dryMOI();
                dry_mois = [dry_moi_matrix(1,1), dry_moi_matrix(2,2), dry_moi_matrix(3,3)];
                text(0.5, 3, 'Dry MOIs (kgm^2): ');
                text(6.5, 3, num2str(dry_mois));

                altitudes = Environment.get_altitudes();
                windspeed_on_ground = Environment.get_windspeed(altitudes(1));
                text(0.5, 4, 'Ground Windspeed (m/s): ');
                text(9, 4, num2str(windspeed_on_ground));

                set(gca,'XTick',[], 'YTick', []);

                % % % % % % % % % % % %
                % External Input Moments %
                subplot(3,4,10)
                plot(log.loggedData.time,log.loggedData.ME_b, 'LineWidth', 1.5);
                hold on
                title('Manual External Moment Inputs')
                ylabel('ME_b (m/s)','FontSize', 12)
                xlabel('Time (s)','FontSize', 12)
                legend('M_x', 'M_y', 'M_z', 'Location', 'northeast')
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
                % Controller Response Moments %
                subplot(3,4,11)
                plot(log.loggedData.time, log.loggedData.MT_b)
                legend('M_{roll}', 'M_{pitch}', 'M_{yaw}', 'Location', 'northeast')
                title('Controller Response Moment')
                ylabel('Controller Moment (Nm)')
                xlabel('Time (s)')
                grid on
                % % % % % % % % % % % %
                % Aerodynamic Moments %
                subplot(3,4,12)
                plot(log.loggedData.time,log.loggedData.MA_b, 'LineWidth', 1.5);   
                title('Aerodynamic Moment')
                ylabel('Moment (Nm)','FontSize', 12)
                xlabel('Time (s)','FontSize', 12)
                grid on
                legend('MA_x','MA_y','MA_z', 'Location', 'northwest')

            end
        end

        function status = stopSolver(obj, ~, ~, flag, startTime, maxTime)
            if strcmp(flag, 'init')  % Solver initialization case
                status = 0;
                return;
            end
        
            elapsedTime = toc(startTime);  % Get elapsed time
        
            if elapsedTime > maxTime
                disp('Terminating ODE solver: Time limit exceeded.');
                obj.odeSolverTerminationFlag = true;
                status = 1;  % Stop solver
            else
                status = 0;  % Continue solver
            end
        end

        function [traj_t, traj_x] = timestepper(obj, x0, dt, tspan, RocketObj, EnvironmentObj, AerodynamicsObj, ControlObj)

            traj_x = x0';
            traj_t = tspan;
            x = x0';
            FlightObject = obj;

            passed_quarter = false;
            passed_half = false;
            passed_threequarter = false;
            for i = 2:length(tspan)
                curr_t = tspan(i);
                k1 = rocketModel(curr_t, x, RocketObj, EnvironmentObj, FlightObject, AerodynamicsObj, ControlObj);
                k2 = rocketModel(curr_t + dt/2, x + (k1')*dt/2, RocketObj, EnvironmentObj, obj, AerodynamicsObj, ControlObj);
                k3 = rocketModel(curr_t + dt/2, x + (k2')*dt/2, RocketObj, EnvironmentObj, obj, AerodynamicsObj, ControlObj);
                k4 = rocketModel(curr_t + dt, x + (k3')*dt, RocketObj, EnvironmentObj, obj, AerodynamicsObj, ControlObj);

                x_next = x + dt*((1/6)*k1' + (1/3)*k2' + (1/3)*k3' + (1/6)*k4');
                traj_x = [traj_x; x_next];
                x = x_next;

                if (x(4) > 2500) & (abs(x(9)) > 1000)
                    disp('Solver likely experienced numerical instability. Consider decreasing time step.')
                    return;
                end

                if i >= 0.25*length(tspan) & ~passed_quarter
                    disp('Simulation 25% complete.')
                    passed_quarter = true;
                end
                if i >= 0.5*length(tspan) & ~passed_half
                    disp('Simulation 50% complete.')
                    passed_half = true;
                end
                if i >= 0.75*length(tspan) & ~passed_threequarter
                    disp('Simulation 75% complete')
                    passed_threequarter = true;
                end
            end
        end


    end

    methods (Static)
        function [value, isterminal, direction] = rocket_events(t, x)
            % rocket_events - Event function for ODE solver
            % Determines when the rocket reaches the ground (px = 0)
            value = x(1);          % Event condition (e.g., altitude)
            isterminal = 1;        % Stop integration when event occurs
            direction = -1;        % Detect only when altitude is decreasing
        end
    end
end
