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
        K 
        q 
        z
        h

        gpsRawData
        gyroRawData
        accRawData
        
        cIBprior
        cBI
        cIB

        accelerometer
        gyroscope
        gps

        covHistory = struct('Px', [], 'Py', [], 'Pz', [], ...
                        'Vx', [], 'Vy', [], 'Vz', [], ...
                        'time', []);
        accelHistory = struct('ax', [], 'ay', [], 'az', [], 'time', []);

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
            if currentTime == 0
                obj.q = Utils.eulerToQuat([0, -90, 0]);
                obj.x = state(2:7);
                ekfState = state;
                return
            end
            obj.getRawData(state, stateDot);

            obj.cIBprior = Utils.dirCosine(obj.q)';

            obj.runAttitudeEstimation();

            obj.cBI = Utils.dirCosine(obj.q);
            obj.cIB = obj.cBI.';
            
            obj.ekf();

            % TEMP REMOVE

            accBody = stateDot(5:7);    


            obj.accelHistory.ax(end+1) = accBody(1);
            obj.accelHistory.ay(end+1) = accBody(2);
            obj.accelHistory.az(end+1) = accBody(3);

            if isempty(obj.accelHistory.time)
                obj.accelHistory.time(1) = obj.currentTime;
            else
                obj.accelHistory.time(end+1) = obj.currentTime;
            end

      
            ekfState = [state(1); obj.x(:); obj.q(:); state(12:14)];

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
            velI = obj.cIB * obj.x(4:6); % Change to cIBprior

            obj.x(1) = obj.x(1) + velI(1) * obj.dt + accI(1) * ((obj.dt^2)/2);
            obj.x(2) = obj.x(2) + velI(2) * obj.dt + accI(2) * ((obj.dt^2)/2);
            obj.x(3) = obj.x(3) + velI(3) * obj.dt + accI(3) * ((obj.dt^2)/2);
            velIx = velI(1) + accI(1) * obj.dt;
            velIy = velI(2) + accI(2) * obj.dt;
            velIz = velI(3) + accI(3) * obj.dt;

            obj.x(4:6) = obj.cBI * [velIx; velIy; velIz];

        end

        function K = kalmanGain(obj)

            H = obj.dhdx(obj.x);
            K = obj.P * H' / (H * obj.P * H' + obj.R);

        end

        function obj = updateState(obj)

            obj.z = obj.gpsRawData';
            y = obj.z - obj.h(obj.x);
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

        %TEMP
                function plotAccelerations(obj)
            t = obj.accelHistory.time;
            if isempty(t)
                error('No acceleration history recorded. Make sure getStateEstimate(...) is being called with stateDot or accelerometer data.');
            end

            ax = obj.accelHistory.ax;
            ay = obj.accelHistory.ay;
            az = obj.accelHistory.az;

            figure;
            plot(t, ax, 'LineWidth', 1.5); hold on;
            plot(t, ay, 'LineWidth', 1.5);
            plot(t, az, 'LineWidth', 1.5);
            grid on;
            xlabel('Time [s]');
            ylabel('Acceleration [m/s^2]');
            legend('a_x (body)', 'a_y (body)', 'a_z (body)', 'Location', 'best');
            title('Body-Frame Accelerations (indices 5:7 of stateDot)');
        end

    end
end
