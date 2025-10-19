classdef Navigation
    % Sensor Modeling & State Estimation
    % 

    properties
        
        dt
        x = zeros(6,1);
        P
        dfdx
        dhdx = @(x)[1 0 0 0 0 0;
                    0 1 0 0 0 0;
                    0 0 1 0 0 0];
        Q
        R = [8.4 0 0;
             0 4.2 0;
             0 0 4.2];
        K 
        q 
        z
        h = @(x)[x(1);x(2);x(3)];
        gyroRawData
        accRawData

        accelerometer
        gyroscope
        gps

    end

    methods
        function obj = Navigation(nav_params, gps, gyroscope, accelerometer)
            
            obj.dt = nav_params.dt;
            obj.P = nav_params.P;
            obj.dfdx = nav_params.dfdx;
            obj.Q = nav_params.Q;
            
            obj.gps = gps;
            obj.gyroscope = gyroscope;
            obj.accelerometer = accelerometer;
            
        end

        function updatedState = runNavigation(obj, state, stateDot)
            
            obj.getRawData(state, stateDot);
            obj.runAttitudeEstimation();
            obj.x = state(1:6);


            obj.ekf();
            
            updatedState = [obj.x; obj.q; state(11:13)];

        end

         function obj = runAttitudeEstimation(obj) % Check what the dt is
            wx = obj.gyroRawData(1);
            wy = obj.gyroRawData(2);
            wz = obj.gyroRawData(3);
            q_delt = obj.gyroToInstantaneousRotationQuat(wx, wy, wz, obj.dt);
            q_new = obj.quatMulitply(obj.q, q_delt);
            obj.q = q_new;

        end

        %% EKF Functions

        function obj = ekf(obj)

            % Sensor Data
            

            % Predict Step
            obj.predictState() 
            obj.P = obj.predictCovariance();

            % Update Step
            % sensor data for this step -----------> sensor models
            obj.K = kalmanGain();
            obj.x = obj.updateState();
            obj.P = updateCovariance();

        end

        function obj = predictState(obj)

            accI = obj.accBodytoInertial();
            obj.x(1) = obj.x(1) + Utils.quatrotate(conj(obj.q),obj.x(4))*obj.dt + accI(1) * ((obj.dt^2)/2);
            obj.x(2) = obj.x(2) + Utils.quatrotate(conj(obj.q),obj.x(5))*obj.dt + accI(2) * ((obj.dt^2)/2);
            obj.x(3) = obj.x(3) + Utils.quatrotate(conj(obj.q),obj.x(6))*obj.dt + accI(3) * ((obj.dt^2)/2);
            obj.x(4) = obj.x(4) + Utils.quatrotate(obj.q,accI(1)) * obj.dt; 
            obj.x(5) = obj.x(5) + Utils.quatrotate(obj.q,accI(2)) * obj.dt; % WRONG WROOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOn
            obj.x(6) = obj.x(6) + Utils.quatrotate(obj.q,accI(3)) * obj.dt;

        end

        function K = kalmanGain(obj)

            H = obj.dhdx(obj.x);
            K = obj.P * H' / (H * obj.P * H' + obj.R);

        end

        function obj = updateState(obj)

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
        % Need to interp1(extrap) --> Vector of times we got sensor data,
        % vector of sensor data, current time we want to interpolate to
        % 
        function obj = getRawData(obj, state, stateDot)
            obj.z = obj.gps.getMeasurement(state,stateDot); % eventually fix
            obj.accRawData = obj.accelerometer.getMeasurment(state,stateDot);
            obj.gyroRawData = obj.gyroscope.getMeasuremtn(staqte,stateDot);
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

        function [xNED, yNED, zNED] = GPS2NED(obj, lat, lon, alt)
            
            [xECEF, yECEF, zECEF] = LLA2ECEF(lat, lon, alt);

            refECEF = LLA2ECEF(obj.gps.refLLA(1), obj.gps.refLLA(2), obj.gps.refLLA(3));
            dx = xECEF - refECEF(1);
            dy = yECEF - refECEF(2);
            dz = zECEF - refECEF(3);

            phi = deg2rad(lat);
            lambda = deg2rad(lon);

            xNED = -sin(phi) * cos(lambda) * dx - sin(phi) * sin(lambda) * dy + cos(phi) * dz;
            yNED = -sin(lambda) * dx + cos(lambda) * dy;
            zNED = -cos(phi) * cos(lambda) * dx - cos(phi) * sin(lambda) * dy - sin(phi) * dz;

        end

        function [accI] = accBodytoInertial(obj)
            accI = Utils.quatrotate(conj(obj.q), obj.accRawData) + [0,0,9.81];
        end
    
        function [q_delt] = gyroToInstantaneousRotationQuat(obj, wx, wy, wz) % dt for now -> change to timeStep of sensor

            omegaVec = [wx wy wz];
            
            if norm(omegaVec) == 0
                omegaVec = omegaVec + 0.001;
            end
            
            axis = omegaVec./norm(omegaVec);
            angle = obj.dt*norm(omegaVec);
            
            q_delt_s = cos(angle/2);
            q_delt_x = axis(1)*sin(angle/2);
            q_delt_y = axis(2)*sin(angle/2);
            q_delt_z = axis(3)*sin(angle/2);
            
            q_delt = [q_delt_s; q_delt_x; q_delt_y; q_delt_z];
            
       end
        
       function [quat_new]= quatMulitply(obj, q1,q2)
            
            q_s_new = q1(1)*q2(1) - q1(2)*q2(2)- q1(3)*q2(3)- q1(4)*q2(4);
            q_ang_new = q1(1).*q2(2:4) + q2(1).*q1(2:4) + cross(q1(2:4), q2(2:4));
                
            quat_new = [q_s_new; q_ang_new];
                
       end
    end
end
