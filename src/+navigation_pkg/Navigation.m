classdef Navigation
    % Sensor Modeling & State Estimation
    % 

    properties
        
        dt
        x = zeros(6,1);
        P
        dfdx
        dhdx
        Q
        R
        K 
        q = [1;0;0;0];
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

        function updatedState = runNavigation(state, stateDot)
            obj.x = state(1:6);
            obj.q = state(7:10);

        end

        %% EKF Functions

        function obj = ekf(obj)

            % Sensor Data
            

            % Predict Step
            obj.predictState(accI) 
            obj.P = obj.predictCovariance();

            % Update Step
            z = 0; % sensor data for this step -----------> sensor models
            obj.K = kalmanGain();
            obj.x = obj.updateState();
            obj.P = updateCovariance();

        end

        function obj = predictState(obj, accI)

            obj.x(1) = obj.x(1) + obj.x(4)*obj.dt + accI(1) * ((obj.dt^2)/2);
            obj.x(2) = obj.x(2) + obj.x(5)*obj.dt + accI(2) * ((obj.dt^2)/2);
            obj.x(3) = obj.x(3) + obj.x(6)*obj.dt + accI(3) * ((obj.dt^2)/2);
            obj.x(4) = obj.x(4) + accI(1) * obj.dt; 
            obj.x(5) = obj.x(5) + accI(2) * obj.dt; % WRONG WROOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOn
            obj.x(6) = obj.x(6) + accI(3) * obj.dt;

        end

        function K = kalmanGain(obj)

            H = obj.dhdx(obj.x);
            K = obj.P * H' / (H * obj.P * H' + obj.R);

        end

        function obj = updateState(obj)

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

            refECEF = LLA2ECEF(obj.refLLA(1), obj.refLLA(2), obj.refLLA(3));
            dx = xECEF - refECEF(1);
            dy = yECEF - refECEF(2);
            dz = zECEF - refECEF(3);

            phi = deg2rad(lat);
            lambda = deg2rad(lon);

            xNED = -sin(phi) * cos(lambda) * dx - sin(phi) * sin(lambda) * dy + cos(phi) * dz;
            yNED = -sin(lambda) * dx + cos(lambda) * dy;
            zNED = -cos(phi) * cos(lambda) * dx - cos(phi) * sin(lambda) * dy - sin(phi) * dz;

        end

        function [accI] = accBodytoInertial(accBody)
            accI = Utils.quatrotate(conj(qBI), accBody) + [0,0,9.81];
        end

    end
end
