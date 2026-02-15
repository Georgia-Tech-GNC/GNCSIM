classdef RollRatePID < controllerPkg.Controller
    properties
        % breakPoints required by Flight scheduling logic even if unused
        breakPoints
        useActuatorModel = true
        referenceAttitudes = []

        % Gains / timing
        Kp
        Ki
        Kd
        dt

        % Limits / anti-windup
        tauMax
        intMin
        intMax

        % State
        intErr = 0
        prevErr = 0

        % Indices (constants)
        idx = StateIndices
    end

    methods
        function obj = RollRatePID(Kp, Ki, Kd, dt, tauMax, intMin, intMax)
            obj.Kp = Kp; obj.Ki = Ki; obj.Kd = Kd;
            obj.dt = dt;
            obj.tauMax = tauMax;
            obj.intMin = intMin;
            obj.intMax = intMax;

            % Never switch schedules (repo expects this field to exist)
            obj.breakPoints = struct('cumVelocityInterval', obj.dt);
        end

        function u = getControl(obj, state, ~, ~)
            % Roll rate (rad/s)
            wx = state(obj.idx.omega_x);

            % Reference roll rate (rad/s)
            wxRef = 0;

            % Error (reference - measurement)
            err = wxRef - wx;

            % Derivative on error
            derr = (err - obj.prevErr) / obj.dt;

            % Candidate integrator update (clamped)
            newIntErr = obj.intErr + err * obj.dt;
            newIntErr = min(max(newIntErr, obj.intMin), obj.intMax);

            % Unsaturated torque command
            tauUnsat = obj.Kp * err + obj.Ki * newIntErr + obj.Kd * derr;

            % Saturate to actuator capability
            tau = min(max(tauUnsat, -obj.tauMax), obj.tauMax);

            % Conditional integration anti-windup:
            % If saturated and error would drive further into saturation, do not integrate.
            satHigh = (tau >= obj.tauMax) && (err > 0);
            satLow  = (tau <= -obj.tauMax) && (err < 0);

            if ~(satHigh || satLow)
                obj.intErr = newIntErr;
            end

            % Update error memory
            obj.prevErr = err;

            % Repo convention: 3x1 [roll; pitch; yaw]
            u = [tau; 0; 0];
        end

        function populateBreakPoints(~)
            % No gain scheduling
        end
    end
end