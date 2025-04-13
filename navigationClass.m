classdef Navigation
    properties
        dt
        EKF_x
        EKF_P
        EKF_Q
        EKF_R
        dfdx
        dhdx
        q_current
        sensedData
        z
        f
        h
        K
        ekfIndex
        accelerometer
        gyro
        gps
        accelForStep
        gpsDataForStep
        gyroForStep
    end
    
    methods
        %Constructor
        function obj = Navigation(initialState,initialCovariance, initialQuaternion, processNoise, measurementNoise, f, dfdx, h, dhdx,accelerometer, gyro, gps)         
            obj.EKF_x = initialState;
            obj.EKF_P = initialCovariance;
            obj.EKF_Q = processNoise; 
            obj.EKF_R = measurementNoise; % For now
            obj.q_current = initialQuaternion;
            obj.f = f;
            obj.dfdx = dfdx;
            obj.dhdx = dhdx;
            obj.h = h;
            obj.accelerometer = accelerometer;
            obj.gyro = gyro;
            obj.gps = gps;
        end
        
        %% EKF Function
        function obj = runEKF(obj, dt)                
                % Prediction Step
                obj.EKF_x = obj.f(obj.EKF_x, obj.accelForStep, dt, velFlat); % Need to get velFlat
                obj.EKF_P = obj.predict_covariance(obj.accelForStep);
                
                % Update Step
                obj.z = obj.gpsDataForStep'; % Need GPS data
                obj.K = obj.kalman_gain();
                obj.EKF_x = obj.update_state();
                obj.EKF_P = obj.update_covariance();
        end
        
        % Attitude Update
        function obj = runAttitudeEstimation(obj,dt)
            wx = obj.gyroForStep(1);
            wy = obj.gyroForStep(2);
            wz = obj.gyroForStep(3);
            q_delt = gyroToInstantaneousRotationQuat(wx, wy, wz, dt);
            q_new = quatMulitply(obj.q_current, q_delt);
            obj.q_current = q_new;

        end
        %% Helper Functions
        % Kalman Gain
        function K = kalman_gain(obj)
            H = obj.dhdx(obj.EKF_x);
            K = obj.EKF_P * H' / (H * obj.EKF_P * H' + obj.EKF_R);
        end
        
        % Update State
        function x_updated = update_state(obj)
            y = obj.z - obj.h(obj.EKF_x);
            x_updated = obj.EKF_x + obj.K * y;
        end
        
        % Update Covariance
        function P_updated = update_covariance(obj)
            H = obj.dhdx(obj.EKF_x);
            P_updated = (eye(length(obj.EKF_x)) - obj.K * H) * obj.EKF_P;
        end
        
        % Predict Covariance
        function P_next = predict_covariance(obj)
            F = obj.dfdx(obj.EKF_x, obj.q_current);
            P_next = F * obj.EKF_P * F' + obj.EKF_Q;
        end
        
        function [accelForStep, gyroForStep, gpsDataForStep] = readSensors(log,t)
            t_ref = log.loggedData.time;

            a_f = log.loggedData.accelFlat;
            q_bf = log.loggedData.q_bf;
            w_bf = log.loggedData.angVel;
            accelData = obj.accelerometer.accelerometer_reading(a_f, w_bf, q_bf);
            accelForStep = interp1(t_ref, accelData, t, 'linear', 'extrap');    

            w_bf = log.loggedData.angVel;
            gyroData = obj.gyro.gyro_model(w_bf, t_ref);
            gyroForStep  = interp1(t_ref, gyroData,t,'linear','extrap');   
            
            gpsData = []; % Still Need a GPS Sensor
            gpsDataForStep = interp1(t_ref, gpsData,t,'linear', 'extrap');

        end

        function [q_delt] = gyroToInstantaneousRotationQuat(wx, wy, wz, timeStep)

            omegaVec = [wx wy wz];
            
            if norm(omegaVec) == 0
                omegaVec = omegaVec + 0.001;
            end
            
            axis = omegaVec./norm(omegaVec);
            angle = timeStep*norm(omegaVec);
            
            q_delt_s = cos(angle/2);
            q_delt_x = axis(1)*sin(angle/2);
            q_delt_y = axis(2)*sin(angle/2);
            q_delt_z = axis(3)*sin(angle/2);
            
            q_delt = [q_delt_s; q_delt_x; q_delt_y; q_delt_z];
            
            end
            
            function [quat_new]= quatMulitply(q1,q2)
            
            q_s_new = q1(1)*q2(1) - q1(2)*q2(2)- q1(3)*q2(3)- q1(4)*q2(4);
            q_ang_new = q1(1).*q2(2:4) + q2(1).*q1(2:4) + cross(q1(2:4), q2(2:4));
            
            quat_new = [q_s_new; q_ang_new];
            
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
    end
end