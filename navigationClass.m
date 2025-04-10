classdef navigationClass
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
        gyroBias
        gyroRange
        gyroNoise_density
        gyroSampling_rate
        accelForStep
        gyroForStep
        gpsData
        t_ref
    end
    
    methods
        %Constructor
        function obj = navigationClass(initialState,initialCovariance, processNoise, initialQuaternion, sensedData,f, dfdx, h, dhdx, dt, gyroBias, gyroRange, gyroNoise_density,gyroSampling_rate, gyroSampling_dt)         
            obj.EKF_x = initialState;
            obj.EKF_P = initialCovariance;
            obj.EKF_Q = processNoise; 
            obj.EKF_R = eye(3); % For now
            obj.q_current = initialQuaternion;
            obj.sensedData = sensedData; 
            obj.ekfIndex =1;
            obj.f = f;
            obj.dfdx = dfdx;
            obj.dhdx = dhdx;
            obj.h = h;
            obj.t_ref = log.loggedData.time;
            obj.dt = log.loggedData.time(2)- log.loggedData.time(1);

            obj.gyroBias = gyroBias;
            obj.gyroRange = gyroRange;
            obj.gyroNoise_density = gyroNoise_density;
            obj.gyroSampling_rate = gyroSampling_rate;
        end
        
        %% EKF Function
        function obj = runEKF(obj)
                % Update sensor data
                [obj.accelForStep, obj.gyroForStep, obj.gpsData] = readSensors(log); % GPS Data is not final still

                
                % Prediction Step
                obj.EKF_x = obj.f(obj.EKF_x, obj.accelForStep, obj.dt);
                obj.EKF_P = obj.predict_covariance();
                
                % Update Step
                obj.z = obj.gps'; % Need GPS data
                obj.K = obj.kalman_gain();
                obj.EKF_x = obj.update_state();
                obj.EKF_P = obj.update_covariance();
            
                obj.ekfIndex = obj.ekfIndex+1;
        end
        
        % Attitude Update
        function obj = runAttitudeEstimation(obj)
            q_delt = obj.gyroToInstantaneousRotationQuat(obj.gyroForStep(1), obj.gyroForStep(2), obj.gyroForStep(3));
            obj.q_current = obj.quatMulitply(obj.q_current, q_delt);

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
            F = obj.dfdx(obj.EKF_x, obj.accelForStep);
            P_next = F * obj.EKF_P * F' + obj.EKF_Q;
        end
        
        function [accelForStep, gyroForStep, gpsData] = readSensors(log)

            a_f = log.loggedData.accelFlat;
            q_bf = log.loggedData.q_bf;
            w_bf = log.loggedData.angVel;
            accelerometer = Sensor();
            accelData = accelerometer.accelerometer_reading(a_f, w_bf, q_bf);
            accelForStep = accelData(obj.ekfIndex);
            
            w_bf = log.loggedData.angVel;
            gyro = Gyro(obj.gyroBias, obj.gyroRange, obj.gyroNoise_density, obj.gyroSampling_rate);
            gyroData = gyro.gyro_model(w_bf, obj.t_ref);
            gyroForStep = gyroData(obj.ekfIndex);
            
            gpsData = obj.sensedData.posFlat(efkIndex,:); % Still Need a GPS Sensor

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