classdef TVCPID < controllerPkg.Controller
    properties
        Kp (3,1) double
        Ki (3,1) double
        Kd (3,1) double
        integralError (3,1) double = zeros(3,1)
        lastTime      (1,1) double = 0
    end

    methods
        function obj = TVCPID(referenceAttitudes, Kp, Ki, Kd, useActuatorModel, breakpoint)
            obj.referenceAttitudes = referenceAttitudes;
            obj.Kp                 = Kp;
            obj.Ki                 = Ki;
            obj.Kd                 = Kd;
            obj.useActuatorModel   = useActuatorModel;
            obj.breakPoints        = breakpoint;
        end

        function rocketControl = getControl(obj, state, currentCumVelocity, currentTime)
            arguments
                obj
                state              (:,1) double
                currentCumVelocity (1,1) double {mustBeNonnegative}
                currentTime        (1,1) double {mustBeNonnegative}
            end

            dt = currentTime - obj.lastTime;
            if dt <= 0; dt = 1e-4; end

            err = obj.getError(state, currentTime);

            obj.integralError = obj.integralError + err * dt;

            omega = state(12:14);

            rocketControlRad = obj.Kp .* err ...
                             + obj.Ki .* obj.integralError ...
                             - obj.Kd .* omega;

            if currentTime < 12
                fprintf('t=%.2f err=[%.3f %.3f %.3f] u=[%.3f %.3f %.3f]\n', ...
                    currentTime, err(1), err(2), err(3), ...
                    rocketControlRad(1), rocketControlRad(2), rocketControlRad(3));
            end

            obj.lastTime = currentTime;
            rocketControl = rad2deg(rocketControlRad);
        end

        function err = getError(obj, rocketStateEstimate, currentTime)
            arguments
                obj
                rocketStateEstimate (:,1) double
                currentTime         (1,1) double {mustBeNonnegative}
            end

            % Normalize Current Quaternion
            q_cur = Utils.normalizeQuat(rocketStateEstimate(8:11));

            % Reference quaternion at current time
            time_vec = obj.referenceAttitudes(1, :);
            [~, idx] = min(abs(time_vec - currentTime));
            q_ref = Utils.normalizeQuat(obj.referenceAttitudes(2:5, idx));

            % Conjugate of q_cur
            q_cur_inv = [q_cur(1); -q_cur(2); -q_cur(3); -q_cur(4)];

            % Rotate body x axis by current and reference quaternions
            vb = [1; 0; 0];
            v_curr    = obj.quatRotate(q_cur(2:4), q_cur(1), vb);
            v_desired = obj.quatRotate(q_ref(2:4), q_ref(1), vb);

            % Error quaternion
            d = dot(v_curr, v_desired);
            a = cross(v_curr, v_desired);

            q_err = [1 + d; a(1); a(2); a(3)];
            q_err = q_err / norm(q_err);

            % Rotate error vector part into body frame
            error_body = obj.quatRotate(q_cur_inv(2:4), q_cur_inv(1), q_err(2:4));

            % Scale Error
            err = [
                error_body(1) * 2;
                error_body(2) * 2;
                error_body(3) * 2
            ];
        end

        function v_rotated = quatRotate(obj, qv, qw, v)
            t         = 2.0 * cross(qv, v);
            v_rotated = v + (qw * t) + cross(qv, t);
        end
    end
end