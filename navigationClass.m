classdef navigationClass
    properties
        t
        t_prev
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
        h
        K
        gps
        accelerometer
        gyro
        ekfIndex
    end
    
    methods
        %Constructor
        function obj = navigationClass(initialState,initialCovariance, processNoise, initialQuaternion, sensedData)
            obj.t = 0;
            obj.t_prev = 0;
            obj.dt = 0;
            
            obj.EKF_x = initialState; % Will change based on how we named it is config
            obj.EKF_P = initialCovariance; % Will change based on how we named it is config
            obj.EKF_Q = processNoise; % Will change based on how we named it is config
            obj.EKF_R = eye(3); % For now
            obj.q_current = initialQuaternion; % Will change based on how we named it is config
            obj.sensedData = sensedData; % Will change based on how we named it is config
            obj.ekfIndex =1;
        end
        
        %% EKF Function
        function obj = runEKF(obj)

                % Get current time and calculate time step
                obj.t = obj.sensedData.time(obj.ekfIndex); % current time based on index
                
                if obj.ekfIndex > 1 % makes sure there is an index before so that we can find time step
                    obj.dt = obj.t - obj.t_prev;  % Time step
                end
                
                obj.t_prev = obj.t;
                
                % Update sensor data for current step
                obj.gps = read_GPS(obj.ekfIndex);
                obj.accelerometer = read_accelerometer(obj.ekfIndex);
                obj.gyro = read_gyro(obj.ekfIndex);
                
                % Prediction Step
                obj.EKF_x = obj.state_transition_fn();
                obj.dfdx = obj.state_transition_jacob();
                obj.EKF_P = obj.predict_covariance();
                
                % Update Step
                obj.z = obj.gps';
                obj.h = obj.observation_fn(); 
                obj.dhdx = obj.observation_jacob();
                obj.K = obj.kalman_gain();
                obj.EKF_x = obj.update_state();
                obj.EKF_P = obj.update_covariance();
            
            disp('EKF process completed.');
        end
        
        % Attitude Update
        function obj = runAttitudeEstimation(obj)
            q_delt = obj.gyroToInstantaneousRotationQuat(obj.gyro(1), obj.gyro(2), obj.gyro(3));
            obj.q_current = obj.quatMultiply(obj.q_current, q_delt);

        end
        %% Helper Functions
        % State Transition Function
        function f = state_transition_fn(obj)
            % Calculates next state for translational EKF
            a_b = obj.accelerometer; % this is true acceleration
            
            velFlat_n = obj.quat_rotate(obj.quatconj(obj.q_current'), [obj.EKF_x(2) obj.EKF_x(4) obj.EKF_x(6)]);
            
            f1 = obj.EKF_x(1) + velFlat_n(1)*obj.dt;
            f2 = obj.EKF_x(2) + obj.dt*a_b(1);
            f3 = obj.EKF_x(3) + velFlat_n(2)*obj.dt;
            f4 = obj.EKF_x(4) + obj.dt*a_b(2);
            f5 = obj.EKF_x(5) + velFlat_n(3)*obj.dt;
            f6 = obj.EKF_x(6) + obj.dt*a_b(3);
            
            f = [f1;f2;f3;f4;f5;f6];
        end
        
        % State Transition Jacobian
        function dfdx = state_transition_jacob(obj)
            % Calculates state transition jacobian for translational EKF
            q_fb = obj.quatconj(obj.q_current');
            
            q0 = q_fb(1);
            q1 = q_fb(2);
            q2 = q_fb(3);
            q3 = q_fb(4);
            
            dfdx = [1, obj.dt*(2*q0^2 + 2*q1^2 - 1), 0,   obj.dt*(2*q0*q3 + 2*q1*q2), 0,  -obj.dt*(2*q0*q2 - 2*q1*q3)
                    0,                        1, 0,                        0, 0,                        0
                    0,  -obj.dt*(2*q0*q3 - 2*q1*q2), 1, obj.dt*(2*q0^2 + 2*q2^2 - 1), 0,   obj.dt*(2*q0*q1 + 2*q2*q3)
                    0,                        0, 0,                        1, 0,                        0
                    0,   obj.dt*(2*q0*q2 + 2*q1*q3), 0,  -obj.dt*(2*q0*q1 - 2*q2*q3), 1, obj.dt*(2*q0^2 + 2*q3^2 - 1)
                    0,                        0, 0,                        0, 0,                        1];
        end
        
        % Observation Function
        function h = observation_fn(obj)
            % Calculates state-to-measurement relation for translational EKF
            h1 = obj.EKF_x(1);
            h2 = obj.EKF_x(3);
            h3 = obj.EKF_x(5);
            
            h = [h1;h2;h3];
        end
        
        % Observation Jacobian
        function dhdx = observation_jacob()
            dhdx = [1, 0, 0, 0, 0, 0;
                    0, 0, 1, 0, 0, 0;
                    0, 0, 0, 0, 1, 0];
        end
        
        % Kalman Gain
        function K = kalman_gain(obj)
            H = obj.dhdx;
            K = obj.EKF_P * H' / (H * obj.EKF_P * H' + obj.EKF_R);
        end
        
        % Update State
        function x_updated = update_state(obj)
            y = obj.z - obj.h;
            x_updated = obj.EKF_x + obj.K * y;
        end
        
        % Update Covariance
        function P_updated = update_covariance(obj)
            H = obj.dhdx;
            P_updated = (eye(length(obj.EKF_x)) - obj.K * H) * obj.EKF_P;
        end
        
        % Predict Covariance
        function P_next = predict_covariance(obj)
            F = obj.dfdx;
            P_next = F * obj.EKF_P * F' + obj.EKF_Q;
        end
        
        function accelData = read_accelerometer(efkIndex)
            accelData = obj.sensedData.accelerometer(efkIndex,:);
        end

        function gyroData = read_gyro(efkIndex)
            gyroData =obj.sensedData.gyro(efkIndex,:);
        end
        
        function gpsData = read_GPS(efkIndex)
            gpsData = obj.sensedData.posFlat(efkIndex,:);
        end


        % Need to Make: 
        %   gyroToInstantaneousRotationQuat
        %   quatMultiply,
        %   quatconj
        %   quat_rotate
    end
end