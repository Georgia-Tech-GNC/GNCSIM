% Main Authors: Conor Foley (mfoley40@gatech.edu)

classdef Navigation < handle
    % Sensor Modeling & State Estimation
    % Frame transform for gps data

    properties (SetAccess = private)
        
        dt
        currentTime 
        x 
        P
        dfdx
        dhdx        
        Q
        R
        q 
        z
        h

        gpsRawData
        gyroRawData
        accRawData
        
        cIBprior
        cBI
        cIB
        qPrior

        accelerometer
        gyroscope
        gps

        covHistory = struct('Px', [], 'Py', [], 'Pz', [], ...
                        'Vx', [], 'Vy', [], 'Vz', [], ...
                        'time', []);

        posHistory = struct('trueX', [], 'trueY', [], 'trueZ', [], ...
                        'gpsX', [], 'gpsY', [], 'gpsZ', [], ...
                        'time', []);

    end

    methods

        function obj = Navigation(navParams, gps, gyroscope, accelerometer)

                obj.dt = navParams.dt;
                obj.P = navParams.P;
                obj.dfdx = navParams.dfdx;
                obj.Q = navParams.Q;
               

                obj.gps = gps;
                obj.gyroscope = gyroscope;
                obj.accelerometer = accelerometer;
                obj.generateObservationMatrices;

        end


        function ekfState = getStateEstimate(obj, state, stateDot, currentTime, log)
            obj.currentTime = currentTime;
            
            % Initial state estimate 
            if currentTime == 0
                obj.q = Utils.eulerToQuat([0, -90, 0]);
                obj.x = state(2:7);
                ekfState = state;
                return
            end

            obj.getRawData(state, stateDot);

            obj.cIBprior = Utils.dirCosine(obj.q)'; 
            obj.qPrior = obj.q;

            obj.runAttitudeEstimation();

            obj.cBI = Utils.dirCosine(obj.q);
            obj.cIB = obj.cBI';

            
            obj.ekf(log);
      
            ekfState = [state(1); obj.x(:); obj.q(:); obj.gyroRawData];

            % disp('State')
            % disp(state)
            % disp("State Dot")
            % disp(stateDot)
            % disp('Ekf')
            % disp(ekfState)
            % disp("Time")
            % disp(currentTime)

            % disp('--- Rotation Matrix Check ---');
            % 
            % % Body axes unit vectors
            % x_body = [1; 0; 0];
            % y_body = [0; 1; 0];
            % z_body = [0; 0; 1];
            % 
            % % Rotate body axes into inertial (ENU) using cIB
            % x_inertial = obj.cIB * x_body;
            % y_inertial = obj.cIB * y_body;
            % z_inertial = obj.cIB * z_body;
            % 
            % disp('Body x-axis in inertial frame:');
            % disp(x_inertial);
            % 
            % disp('Body y-axis in inertial frame:');
            % disp(y_inertial);
            % 
            % disp('Body z-axis in inertial frame:');
            % disp(z_inertial);
            % 
            % % Optional: Check that cIB is orthonormal
            % orthonormality = obj.cIB * obj.cBI; % should be close to identity
            % disp('cIB * cBI (should be identity):');
            % disp(orthonormality);


            obj.posHistory.trueX(end+1) = state(2);
obj.posHistory.trueY(end+1) = state(3);
obj.posHistory.trueZ(end+1) = state(4);

% ---- GPS Position (ENU) ----
if isempty(obj.gpsRawData)
    obj.posHistory.gpsX(end+1) = NaN;
    obj.posHistory.gpsY(end+1) = NaN;
    obj.posHistory.gpsZ(end+1) = NaN;
else
    obj.posHistory.gpsX(end+1) = obj.gpsRawData(1);
    obj.posHistory.gpsY(end+1) = obj.gpsRawData(2);
    obj.posHistory.gpsZ(end+1) = obj.gpsRawData(3);
end

% ---- Time ----
obj.posHistory.time(end+1) = obj.currentTime;
            




        end

        function obj = runAttitudeEstimation(obj)
            wx = obj.gyroRawData(1);
            wy = obj.gyroRawData(2);
            wz = obj.gyroRawData(3);
            qDelt = obj.gyroToInstantaneousRotationQuat(wx, wy, wz);
            qNew = obj.quatMulitply(obj.q, qDelt);
            obj.q = qNew;

        end

        %% EKF Functions

        function obj = ekf(obj, log)

            % Predict Step
            obj.predictState();
            obj.P = obj.predictCovariance();

            % Update Step
            % don't auto set z use observation matrix
            if ~(isempty(obj.gpsRawData))
                [K, H] = obj.kalmanGain();
                obj.updateState(K);
                obj.P = obj.updateCovariance(K, H);
            end

            diagP = diag(obj.P);
            log.add(obj.currentTime, 'ekfcovariances', diagP);
        end

        function obj = predictState(obj)

            accI = obj.accBodytoInertial();
            accB = obj.cBI * accI;
            velI = obj.cIB * obj.x(4:6); % cIB Prior?

            obj.x(1) = obj.x(1) + velI(1) * obj.dt + accI(1) * ((obj.dt^2)/2);
            obj.x(2) = obj.x(2) + velI(2) * obj.dt + accI(2) * ((obj.dt^2)/2);
            obj.x(3) = obj.x(3) + velI(3) * obj.dt + accI(3) * ((obj.dt^2)/2);

            obj.x(4) = obj.x(4) + accB(1) * obj.dt;
            obj.x(5) = obj.x(5) + accB(2) * obj.dt;
            obj.x(6) = obj.x(6) + accB(3) * obj.dt;

        end

        function [K, H] = kalmanGain(obj)

            H = obj.dhdx(obj.x);
            S = H * obj.P * H' + obj.R;
            K =  obj.P * H' / S;         %obj.P * H' / (H * obj.P * H' + obj.R);

        end

        function obj = updateState(obj, K)

            obj.z = obj.gpsRawData';
            y = obj.z - obj.h(obj.x);
            obj.x = obj.x + K * y;

        end

        function P_posterior = updateCovariance(obj, K, H)

            % H = obj.dhdx(obj.x);
            P_posterior = (eye(length(obj.x)) - K * H) * obj.P;

        end

        function P_prior = predictCovariance(obj)

            F = obj.dfdx(obj.dt, obj.qPrior);
            P_prior = F * obj.P * F' + obj.Q;

        end

        %% Sensor Data
        function obj = getRawData(obj, state, stateDot)

            gpsRawDataLLA = obj.gps.getMeasurement(state,stateDot,obj.currentTime);
            if ~(isempty(gpsRawDataLLA))
                [xENU, yENU, zENU] = obj.GPS2ENU(gpsRawDataLLA(1),gpsRawDataLLA(2),gpsRawDataLLA(3));
                obj.gpsRawData = [xENU, yENU, zENU];

            else
                obj.gpsRawData = [];
            end

            obj.accRawData = obj.accelerometer.getMeasurement(state,stateDot,obj.currentTime);

            obj.gyroRawData = obj.gyroscope.getMeasurement(state,stateDot,obj.currentTime);

        end

        function obj = generateObservationMatrices(obj)

            [gpsH, gpsR] = obj.gps.getObservationMatrices();
            obj.dhdx = gpsH;
            obj.R = gpsR;
            obj.h = @(x)[x(1);x(2);x(3)];
        end
        %% Frame Transformation Functions

        % GPS
        function [xECEF, yECEF, zECEF] = LLA2ECEF(obj, lat, lon, alt)
            % Converts from WGS84 to ECEF

            WGS84_E = 0.08181;
            WGS84_A = 6378137.0;   

            % Convert latitude and longitude from degrees to radians
            phi = deg2rad(lat);
            lambda = deg2rad(lon);

            % Convert LLA to ECEF
            N = WGS84_A / sqrt(1 - (WGS84_E^2 * (sin(phi))^2));

            xECEF = (N + alt) * cos(phi) * cos(lambda);
            yECEF = (N + alt) * cos(phi) * sin(lambda);
            zECEF = (N * (1.0 - WGS84_E * WGS84_E) + alt) * sin(phi);

        end

        function [xENU, yENU, zENU] = GPS2ENU(obj, lat, lon, alt)
            
            [xECEF, yECEF, zECEF] = obj.LLA2ECEF(lat, lon, alt);

            [xECEFref, yECEFref, zECEFref] = obj.LLA2ECEF(obj.gps.refLLA(1), obj.gps.refLLA(2), obj.gps.refLLA(3));
            dx = xECEF - xECEFref;
            dy = yECEF - yECEFref;
            dz = zECEF - zECEFref;

            phi = deg2rad(obj.gps.refLLA(1)); % Check these
            lambda = deg2rad(obj.gps.refLLA(2));

            xENU = -sin(lambda)*dx + cos(lambda)*dy;
            yENU = -sin(phi)*cos(lambda)*dx - sin(phi)*sin(lambda)*dy + cos(phi)*dz;
            zENU = cos(phi)*cos(lambda)*dx + cos(phi)*sin(lambda)*dy + sin(phi)*dz;

        end

        function [accI] = accBodytoInertial(obj)

            accI = obj.cIB * obj.accRawData + [0; 0; -9.81];

        end
    
        function [qDelt] = gyroToInstantaneousRotationQuat(obj, wx, wy, wz) 

            omegaVec = [wx wy wz];

            if norm(omegaVec) == 0
                qDelt = [1;0;0;0]; 
                return
            end

            axis = omegaVec./norm(omegaVec);
            angle = obj.gyroscope.outputPeriod*norm(omegaVec);

            qDeltS = cos(angle/2);
            qDeltX = axis(1)*sin(angle/2);
            qDeltY = axis(2)*sin(angle/2);
            qDeltZ = axis(3)*sin(angle/2);

            qDelt = [qDeltS; qDeltX; qDeltY; qDeltZ];

        end

        function [quatNew]= quatMulitply(obj, q1,q2)

            qSNew = q1(1)*q2(1) - q1(2)*q2(2)- q1(3)*q2(3)- q1(4)*q2(4);
            qAngNew = q1(1).*q2(2:4) + q2(1).*q1(2:4) + cross(q1(2:4), q2(2:4));

            quatNew = [qSNew; qAngNew];
            quatNew = quatNew / norm(quatNew);

        end

        function plotPositionHistory(obj)
    % Check if any data exists
    if isempty(obj.posHistory.time)
        warning('No position history to plot.');
        return
    end

    t = obj.posHistory.time;

    figure;
    
    % ---- X ----
    subplot(3,1,1)
    plot(t, obj.posHistory.trueX, 'LineWidth', 1.5); hold on;
    plot(t, obj.posHistory.gpsX, '.', 'MarkerSize', 10);
    ylabel('X (m)');
    title('True Position vs GPS Position');
    legend('True','GPS');
    grid on;

    % ---- Y ----
    subplot(3,1,2)
    plot(t, obj.posHistory.trueY, 'LineWidth', 1.5); hold on;
    plot(t, obj.posHistory.gpsY, '.', 'MarkerSize', 10);
    ylabel('Y (m)');
    legend('True','GPS');
    grid on;

    % ---- Z ----
    subplot(3,1,3)
    plot(t, obj.posHistory.trueZ, 'LineWidth', 1.5); hold on;
    plot(t, obj.posHistory.gpsZ, '.', 'MarkerSize', 10);
    ylabel('Z (m)');
    xlabel('Time (s)');
    legend('True','GPS');
    grid on;
end

    end
end
