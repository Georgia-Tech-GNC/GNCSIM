classdef TVCPID < controllerPkg.Controller
    % TVCPID Represents the PID controller for a TVC rocket

    properties
        referenceAttitudes                      % Reference attitudes [time; q0; q1; q2; q3]
        Kp (3,1) double                         % Proportional gains [roll; pitch; yaw]
        Ki (3,1) double                         % Integral gains [roll; pitch; yaw]
        Kd (3,1) double                         % Derivative gains [roll; pitch; yaw]
        useActuatorModel = false                % boolean to check if the actuator model is being used (type hint in Controller.m)

        % PID internal state
        integralError (3,1) double = zeros(3,1) % Accumulated rotation vector error
        lastTime      (1,1) double = 0          % Time of last getControl
    end

    methods
        % Constructor
        function obj = TVCPID(referenceAttitudes, Kp, Ki, Kd, useActuatorModel)
            obj.referenceAttitudes = referenceAttitudes;
            obj.Kp                 = Kp;
            obj.Ki                 = Ki;
            obj.Kd                 = Kd;
            obj.useActuatorModel   = useActuatorModel;
        end

        function rocketControl = getControl(obj, state, currentCumVelocity, currentTime)
            arguments
                obj
                state              (:,1) double
                currentCumVelocity (1,1) double {mustBeNonnegative}
                currentTime        (1,1) double {mustBeNonnegative}
            end

            % Compute dt
            dt = currentTime - obj.lastTime;
            if dt <= 0; dt = 1e-4; end

            % Compute rotation vector error from quaternion attitude error
            err = obj.getError(state, currentTime);

            % Integral term
            obj.integralError = obj.integralError + err * dt;

            % Derivative term
            omega = state(12:14);

            % PID control
            rocketControlRad = obj.Kp .* err + obj.Ki .* obj.integralError + obj.Kd .* omega;

            % Update internal time
            obj.lastTime = currentTime;

            % Convert to degrees for actuator command
            rocketControl = rad2deg(rocketControlRad);
        end

        function err = getError(obj, rocketStateEstimate, currentTime)
            arguments
                obj
                rocketStateEstimate (:,1) double
                currentTime         (1,1) double {mustBeNonnegative}
            end

            % Get current quaternion
            q_cur = Utils.normalizeQuat(rocketStateEstimate(8:11));

            % Find the index in reference that corresponds to current time
            time_vec = obj.referenceAttitudes(1, :);
            [~, idx] = min(abs(time_vec - currentTime));

            % Normalize reference quaternion at current time
            q_ref = Utils.normalizeQuat(obj.referenceAttitudes(2:5, idx));

            % Compute quaternion error
            q_cur_inv = [q_cur(1); -q_cur(2); -q_cur(3); -q_cur(4)];
            q_err = Utils.quatMult(q_cur_inv, q_ref);

            if q_err(1) < 0
                q_err = -q_err;
            end

            % Get error
            if vec_norm < 1e-20
                err = zeros(3,1);
            else
                theta = 2 * atan2(vec_norm, q_err(1));  % rad
                n_hat = q_err(2:4) / vec_norm;          % unit vector
                err   = theta * n_hat;                  % rotation vector
            end
        end
    end
end