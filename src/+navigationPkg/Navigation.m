classdef Navigation < handle
    % Sensor Modeling & State Estimation
    % Frame transform for gps data

    properties (SetAccess = private)
        
        dt
        currentTime 
        x = zeros(6,1);
        P
        dfdx
        dhdx        
        Q
        R
        K 
        q 
        z
        h

        gpsRawData
        gyroRawData
        accRawData

        cBI
        cIB

        accelerometer
        gyroscope
        gps

        covHistory = struct('Px', [], 'Py', [], 'Pz', [], ...
                        'Vx', [], 'Vy', [], 'Vz', [], ...
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


        function ekfState = getStateEstimate(obj, state, stateDot, currentTime)
            obj.currentTime = currentTime;

            if obj.currentTime == 0
                obj.q = Utils.eulerToQuat([0, -90, 0]);
                ekfState = [state(1); obj.x(:); obj.q; state(12:14)];
                return
            end
            obj.getRawData(state, stateDot);

            obj.cBI = Utils.dirCosine(obj.q);
            obj.cIB = obj.cBI.';

            obj.runAttitudeEstimation();
            
            obj.ekf();
      
            ekfState = [state(1); obj.x(:); obj.q(:); state(12:14)];

            % disp('State')
            % disp(state)
            % disp('Ekf')
            % disp(ekfState)

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

        function obj = ekf(obj)

            % Sensor Data
            

            % Predict Step
            obj.predictState();
            obj.P = obj.predictCovariance();

            % Update Step
            % don't auto set z use observation matrix
            if ~(isempty(obj.gpsRawData))
                obj.K = obj.kalmanGain();
                obj.updateState();
                obj.P = obj.updateCovariance();
            end

            diagP = diag(obj.P);
            obj.covHistory.Px(end+1) = diagP(1);
            obj.covHistory.Py(end+1) = diagP(2);
            obj.covHistory.Pz(end+1) = diagP(3);
            obj.covHistory.Vx(end+1) = diagP(4);
            obj.covHistory.Vy(end+1) = diagP(5);
            obj.covHistory.Vz(end+1) = diagP(6);

            if isempty(obj.covHistory.time)
                obj.covHistory.time(1) = 0;
            else
                obj.covHistory.time(end+1) = obj.covHistory.time(end) + obj.dt;
            end

        end

        function obj = predictState(obj)

            accI = obj.accBodytoInertial();
            velI = obj.cIB * obj.x(4:6);
            accB = obj.cBI * accI;
            obj.x(1) = obj.x(1) + velI(1) * obj.dt + accI(1) * ((obj.dt^2)/2);
            obj.x(2) = obj.x(2) + velI(2) * obj.dt + accI(2) * ((obj.dt^2)/2);
            obj.x(3) = obj.x(3) + velI(3) * obj.dt + accI(3) * ((obj.dt^2)/2);
            obj.x(4) = obj.x(4) + accB(1) * obj.dt;
            obj.x(5) = obj.x(5) + accB(2) * obj.dt;
            obj.x(6) = obj.x(6) + accB(3) * obj.dt;

        end

        function K = kalmanGain(obj)

            H = obj.dhdx(obj.x);
            K = obj.P * H' / (H * obj.P * H' + obj.R);

        end

        function obj = updateState(obj)

            obj.z = obj.gpsRawData;
            y = obj.z - obj.h(obj.x); % Need to do z and h
            obj.x = obj.x + obj.K * y;

        end

        function P_posterior = updateCovariance(obj)

            H = obj.dhdx(obj.x);
            P_posterior = (eye(length(obj.x)) - obj.K * H) * obj.P;

        end

        function P_prior = predictCovariance(obj)

            F = obj.dfdx(obj.dt, obj.q);
            P_prior = F * obj.P * F' + obj.Q;

        end

        %% Sensor Data
        function obj = getRawData(obj, state, stateDot)

            gpsRawDataLLA = obj.gps.getMeasurement(state,stateDot,obj.currentTime);
            if ~(isempty(gpsRawDataLLA))
                [xENU, yENU, zENU] = obj.GPS2ENU(gpsRawDataLLA(1),gpsRawDataLLA(2),gpsRawDataLLA(3));
                obj.gpsRawData = [xENU, yENU, zENU];
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
        function [xECEF, yECEF, zECEF] = LLA2ECEF(lat, lon, alt)
            % Converts from WGS84 to ECEF

            WGS84_E = 0.08181;
            WGS84_A = 6378137.0;   

            % Convert latitude and longitude from degrees to radians
            phi = deg2rad(lat);
            lambda = deg2rad(lon);

            % Convert LLA to ECEF
            N = WGS84_A / sqrt(1 - (WGS84_E * sin(phi))^2);

            xECEF = (N + alt) * cos(phi) * cos(lambda);
            yECEF = (N + alt) * cos(phi) * sin(lambda);
            zECEF = (N * (1.0 - WGS84_E * WGS84_E) + alt) * sin(phi);

        end

        function [xENU, yENU, zENU] = GPS2ENU(obj, lat, lon, alt)
            
            [xECEF, yECEF, zECEF] = LLA2ECEF(lat, lon, alt);

            refECEF = LLA2ECEF(obj.gps.refLLA(1), obj.gps.refLLA(2), obj.gps.refLLA(3));
            dx = xECEF - refECEF(1);
            dy = yECEF - refECEF(2);
            dz = zECEF - refECEF(3);

            phi = deg2rad(lat);
            lambda = deg2rad(lon);

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
                omegaVec = omegaVec + 0.001;
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

        end
       %% Plotting
        function plotCovariances(obj)
            t = obj.covHistory.time;
        
            figure;
            subplot(2,1,1);
            plot(t, obj.covHistory.Px, t, obj.covHistory.Py, t, obj.covHistory.Pz, 'LineWidth', 1.5);
            grid on;
            xlabel('Time [s]');
            ylabel('Position Covariance');
            legend('x', 'y', 'z');
            title('Position Covariances');
        
            subplot(2,1,2);
            plot(t, obj.covHistory.Vx, t, obj.covHistory.Vy, t, obj.covHistory.Vz, 'LineWidth', 1.5);
            grid on;
            xlabel('Time [s]');
            ylabel('Velocity Covariance');
            legend('vx', 'vy', 'vz');
            title('Velocity Covariances');
        end
    end
end
