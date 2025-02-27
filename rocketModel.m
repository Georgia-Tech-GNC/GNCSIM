function [xDot, log] = rocketModel(t, x, RocketObj, EnvironmentObj, FlightObj, AerodynamicsObj, ControlObj)
% rocketModel: Open loop model for jet vanes rocket dynamics
% t: current time
% x: current state: [px py pz vx vy vz q0 q1 q2 q3 wx wy wz]
% RocketObj:  object representing rocket
% AerodynamicsObj: object representing aerodynamic forces on rocket
% FlightObj: object representing flight of the rocket
% ControlObj: an object that denotes the control algorithm running

% xDot = f(x, u)
% log --> data that gets logged

    %% Print time to secondary terminal
    persistent terminalCounter resetTerminalCounter;
    
    if isempty(terminalCounter)
        terminalCounter = 0;
    end
    
    if isempty(resetTerminalCounter)
        resetTerminalCounter = 0;
    end
    
    terminalCounter = terminalCounter + 1;
    resetTerminalCounter = resetTerminalCounter + 1;
    
    if resetTerminalCounter >= 3000
         %tprintf('rocketModel');
         resetTerminalCounter = 0;
    end
    
    if terminalCounter >= 100
        %tprintf('rocketModel',num2str(t));
        terminalCounter = 0;
    end

    %% Extract config variables
    tSpan = FlightObj.get_tSpan();
    uSpan = FlightObj.get_uSpan();
    wetMass = RocketObj.get_wetMass();
    dryMass = RocketObj.get_dryMass();

    % baseIb = config.baseIb;
    % invBaseIb = config.invBaseIb;

    radius = RocketObj.get_radius();
    g = FlightObj.get_gravity();
    burnTime = RocketObj.get_burnTime();
    wetMOI = RocketObj.get_wetMOI();
    dryMOI = RocketObj.get_dryMOI();
    CG0 = RocketObj.get_initialCG();
    finalCG = RocketObj.get_finalCG();
    runClosedLoop = FlightObj.get_runClosedLoop();
    applyWind = FlightObj.get_applyWind();
    %applyPeriodicWindGusts = FlightObj.get_applyPeriodicWindGusts();
    applyPeriodicWindGusts = false;
    mainChuteCd = RocketObj.get_mainChuteCd;
    mainChuteArea = RocketObj.get_mainChuteArea;
    drogueChuteCd = RocketObj.get_drogueChuteCd;
    drogueChuteArea = RocketObj.get_drogueChuteArea;
    controlsTime = FlightObj.get_controlsTime;


    %% Extract states
    % Position in flat Earth frame, "f" (m)
    px = x(1);
    py = x(2);
    pz = x(3);
    p_f = [px;py;pz];

    % Velocity in body frame, "b" (m/s)
    vx = real(x(4));
    vy = real(x(5));
    vz = real(x(6));
    v_b = [vx;vy;vz];
    
    % Unit quaternion that rotates from "f" to "b" (unitless)
    q0 = real(x(7));
    q1 = real(x(8));
    q2 = real(x(9));
    q3 = real(x(10));
    q_bf = quatnormalize([q0 q1 q2 q3]);
    v_f = quat_rotate(quatconj(q_bf),real(v_b)');
    
    % Angular velocity with respect to "f" expressed in "b" (rad/s)
    wx = real(x(11));
    wy = real(x(12));
    wz = real(x(13));
    wbf_b = [wx;wy;wz];

    %% Get intermediate values

    T = RocketObj.getThrust(t); % Get thrust at the instantaneous moment in time
    if t < burnTime
        m = wetMass - ((wetMass - dryMass)/burnTime) * t;
        CG = CG0 - ((CG0 - finalCG)/burnTime) * t;
        Ib = wetMOI - ((wetMOI - dryMOI)/burnTime) .* t;
    else
        m = dryMass;
        CG = finalCG;
        Ib = dryMOI;
    end

    %% Gravity

    FG_v = -m*g*[1;0;0]; % gravity force in vehicle carried frame (aligned with flat earth frame)
    FG_b = quat_rotate(q_bf,FG_v'); % parts(quatconj(q_obj)*quaternion([0;FG_v]')*q_obj);
    FG_b = FG_b';

    %% Thrust
    
    FT_b = [0.95.*T;0;0]; % Decimal preceding *T represents the thrust loss due to jet vanes
    % Confirm actual thrust loss value due to static fires

    %% Closed Loop Controls

    % Note: Controls in the loop for nonlinear sims has typically had extreme performance 
    % issues. 
    if (runClosedLoop && (t > 1) && (t < controlsTime))
        controller_ID = ControlObj.get_controllerID();
        switch controller_ID
            case 'LQR'
                % xRef = interp1(tSpan,xRefSpan,t);

                lin_x = [vx;vy;vz;wx;wy;wz;q1;q2;q3];
                [K, x0, ~] = ControlObj.LQR_configuration_selector(lin_x, t);
                % del_u = r' - K*del_x;

                % Old bug: used to be x0' instead. Quite a devious bug, bc it doesnt error and del_u is just
                % a matrix that can still be referenced

                del_x = lin_x - x0;
                del_u = ControlObj.compute_controls(K, del_x);
                %del_u = -K*del_x;

                MT_bx = del_u(1); % M_roll

                % Forcefully turning off pitch control in the FSW? In that case 
                % M_pitch would be zero in the nonlinear (truth) model
                % MT_by = del_u(2) + u0(2); % M_pitch
                MT_by = 0;
                MT_bz = del_u(3); % M_yaw  
            case 'PID'
                % TODO
            case 'MPC'
                % TODO
        end
    else
        MT_bx = 0;
        MT_by = 0;
        MT_bz = 0;    
    end


    %% Constrain MT_b
    % This is based on static fire data. Needs to be updated to reflect
    % real life!

    if MT_bz > 80
        MT_bz = 80;
    end

    if MT_bz < -80
        MT_bz = -80;
    end

    % if MT_bz > 35 && t >= 10
    %     MT_bz = 35;
    % end
    % 
    % if MT_bz < -35 && t >= 10
    %     MT_bz = -35;
    % end
    MT_b = [MT_bx;MT_by;MT_bz];

    %% Aerodynamics & Environment. TODO: Consult Hridai on how this should work with wind/environment
    
    % Getting Air Density
    altitude = px;
    [T,a,P,rho,nu] = atmosisa(altitude);

    % Applying Wind
    if applyWind
        % Discrete Wind Gust Model
        %[Vm, dm] = EnvironmentObj.get_wind_aspects();

        % Direction of Wind Speed in Flat Earth Frame (Must be unit vector)
        udir = [0 1 0];
        
        

            %{
        if ~applyPeriodicWindGusts
            
            % Distance moved into gust
            normXu = norm(dot([px py pz],udir));
            normXu = normXu + Vm*t; % Assuming wind gust moves at a speed Vm and starts at t = 0. starts at a distance dm away from rocket
            
            % Calculating Wind Speed with Discrete Gust Model
            windspeed = EnvironmentObj.wind_speed_change(Vm,dm,normXu); % This takes in positon assuming that wind gust happens at the beginning of trajectory.
        
        else

            % Gust Period
            T = 2.5; % Seconds Between the Start of Each Gust
        
            % Distance moved into gust (Assuming that rocket position does not affect )
            normXu = Vm*mod(t,T);
            
            % Calculating Wind Speed with Discrete Gust Model
            windspeed = EnvironmentObj.wind_speed_gust(Vm,dm,normXu); % This takes in positon assuming that wind gust happens at the beginning of trajectory.
           
        end
         %}
        windspeed = EnvironmentObj.get_windspeed(altitude);

        % wind gust in y and z
        v_a = v_b + quat_rotate(q_bf,windspeed*udir)'; % This is to account for wind model.
        %v_a = v_b + windspeed*udir

        % Calculating to velocity to account for wind speed
        va_x = v_a(1);
        va_y = v_a(2);
        va_z = v_a(3);
    
        alpha = atan2(real(va_y),real(va_x)); % angle of attack (rad)
        beta = atan2(va_z,(va_x^2 + va_y^2)^0.5); % angle of sideslip (rad) asin(vy/Va)?

        if v_f(1) < 0
            FA_b = [0;0;0]; % Aeroforces and moments turned off for descent
            MA_b = [0;0;0];
        else
            if va_x < 0
                [FA_b, MA_b] = AerodynamicsObj.calcAeroBodyFrame_JV(-1*va_x, va_y, va_z, wx, wz, rho, CG);
            else
                [FA_b, MA_b] = AerodynamicsObj.calcAeroBodyFrame_JV(va_x, va_y, va_z, wx, wz, rho, CG);
            end
        end
    else
        if v_f(1) < 0
            FA_b = [0;0;0]; % Aeroforces and moments turned off for descent
            MA_b = [0;0;0];
        else
            if vx < 0
                [FA_b, MA_b] = AerodynamicsObj.calcAeroBodyFrame_JV(-1*vx, vy, vz, wx, wz, rho, CG);
            else
                [FA_b, MA_b] = AerodynamicsObj.calcAeroBodyFrame_JV(vx, vy, vz, wx, wz, rho, CG);
            end
        end

        alpha = atan2(vy, vx);
        beta = atan2(vz, vx);
    end

    %% Parachute Forces
    if v_f(1) < 0
        FP_b_mag = drogueChuteCd * rho * ((norm(v_b)^2)/2) * drogueChuteArea; % Drogue chute deploys at apogee
        FP_b = FP_b_mag * (-v_b/(norm(v_b)));
    elseif (t > burnTime) && (px < 304.8)
        FP_b_mag = (mainChuteCd + drogueChuteCd) * rho * ((norm(v_b)^2)/2) * (mainChuteArea + drogueChuteArea); % Main chute deploys at 1000 ft (304.8 m)
        FP_b = FP_b_mag * (-v_b/(norm(v_b)));
    else
        FP_b = [0;0;0];
    end
   

    %% External Moments
    % Note: similar performance issues as controls
    if uSpan == 0 % interp1 strong effect on sim perf
        ME_bx = 0;
        ME_by = 0;
        ME_bz = 0;
    else
        ME_bx = interp1(tSpan,uSpan(:,1),t);
        ME_by = interp1(tSpan,uSpan(:,2),t);
        ME_bz = interp1(tSpan,uSpan(:,3),t);

        % NOTE: for f(x,u) do this instead
        % MT_bx = u(1);
        % MT_by = u(2);
        % MT_bz = u(3);
    end
    ME_b = [ME_bx;ME_by;ME_bz];

    %% State Derivatives

    % Currently we are unable to factor out m(t) from the MOI. Factoring
    % out an m(t) is beneficial bc it allows the base MOI to be constant,
    % which speeds up sims. If we ever are able to in the future, use 
    % this trick vvv
    % Ib = m.*baseIb;
    % invIb = (1/m).*invBaseIb;
    
    % position qns of motion
    p_f_dot = quat_rotate(quatconj(q_bf),v_b'); %  parts(q_obj*quaternion([0;V_b]')*quatconj(q_obj));
    p_f_dot = p_f_dot'; % [3x1]
    
    % velocity eqns of motion
    F_b = FG_b + FT_b + FA_b + FP_b;
    v_b_dot = (1/m)*F_b - cross(wbf_b, v_b);
    
    % quaternion eqns of motion
    q0_dot = 0.5 * (-wx*q1 - wy*q2 - wz*q3);
    q1_dot = 0.5 * (wx*q0 + wz*q2 - wy*q3);
    q2_dot = 0.5 * (wy*q0 - wz*q1 + wx*q3);
    q3_dot = 0.5 * (wz*q0 + wy*q1 - wx*q2);
    q_bf_dot = [q0_dot;q1_dot;q2_dot;q3_dot];
    
    % angular rate eqns of motion
    M_b = MT_b + MA_b + ME_b;
    
    %     wbf_b_dot = [0;0;0];
    %     wbf_b_dot(1) = 0;
    %     wbf_b_dot(2) = 0;
    %     wbf_b_dot(3) = M_b(3)/Ib(3,3);

    wbf_b_dot = Ib\(M_b - cross(wbf_b, Ib*wbf_b)); % inv(A)*b == A\b
%   wbf_b_dot = inv(Ib) * (M_b - cross(wbf_b, Ib*wbf_b)); % slower
    %wbf_b_dot = inv(Ib) * (M_b - cross(wbf_b, Ib*wbf_b)); % slower
    
    % stack components
    xDot = [p_f_dot; v_b_dot; q_bf_dot; wbf_b_dot];

    % Kinematic constraints - necessary if initial thrust is lower than mg
    if t < 1 % Only when initially launching...
        if xDot(1) < 0
            xDot(1) = 0; % prevents rocket from sinking lower than its launch altitude
        end
        if xDot(4) < 0
            xDot(4) = 0; % prevents rocket from having AOA = 180 deg
        end
    end
    
    %% Logging

    % add to log
    log.alpha = alpha;
    log.beta = beta;
    % log.D0 = D0;
    % log.L = L;
    % log.Di = Di;
    % log.M_ps = M_ps;
    % log.MT_b = MT_b';
    % log.MA_b = MA_b';
    log.CG = CG;
    % log.FT_b = FT_b';
    % log.F_b = F_b';
    log.MA_b = MA_b';
    log.MT_b = MT_b';
    log.ME_b = ME_b';
    log.FG_b = FG_b';
    log.FT_b = FT_b';
    log.FA_b = FA_b';
    log.FP_b = FP_b';
    log.velFlat = p_f_dot'; % should be [1x3]
    log.accelFlat =quat_rotate(quatconj(q_bf),real(v_b_dot)');
    %log.l_p = l_p;
    %log.n_a = n_a;
    %log.n_r = n_r;
    
end

function vB = quat_rotate(q,vI)
    %quat_rotate Rotates vec data set from inertial to body
    % q - quat data set that rotates from I to B - qBI (nx4)
    % vI - 3d vector in I frame (nx3)
    % vB - 3d vec in B frame (nx3)
    % n - # of data entries
    
    dims = size(vI);
    
    % normalize quat
    q_norm = (q(:,1).^2 + q(:,2).^2 + q(:,3).^2 + q(:,4).^2) .^ 0.5;
    q_u0 = q(:,1) ./ q_norm;
    q_u1 = q(:,2) ./ q_norm;
    q_u2 = q(:,3) ./ q_norm;
    q_u3 = q(:,4) ./ q_norm;
    q_u = [q_u0 q_u1 q_u2 q_u3];
    
    % quat rotation
    [~, vB1, vB2, vB3] = parts(quaternion(quatconj(q_u)) .* quaternion([zeros(dims(1),1) vI]) .* quaternion(q_u));
    vB = [vB1 vB2 vB3];

end