% GNCSIM/src/+controllerPkg/RollRatePID.m
classdef RollRatePID < controllerPkg.Controller
    properties
        % Required by repo scheduling logic even if unused
        breakPoints
        useActuatorModel = true
        referenceAttitudes = []

        % Gains / timing
        Kp
        Ki
        Kd
        dt

        % Full-authority actuator limit (symmetric)
        tauMax   % >0, full-scale clamp is ±tauMax

        % Integral budget fraction (0..1), typically 0.3–0.7
        gamma

        % State (matches C++ semantics)
        I_stored = 0    % stored I contribution in TORQUE units
        prevErr  = 0

        % Indices
        idx = StateIndices
    end

    methods
        function obj = RollRatePID(Kp, Ki, Kd, dt, tauMax, gamma)
            obj.Kp = Kp; obj.Ki = Ki; obj.Kd = Kd;
            obj.dt = dt;

            obj.tauMax = tauMax;
            obj.gamma  = gamma;

            obj.breakPoints = struct('cumVelocityInterval', obj.dt);
        end

        function u = getControl(obj, state, ~, ~, alpha)
            % Default alpha=1 (no derating) if caller doesnt pass it
            if nargin < 5 || isempty(alpha)
                alpha = 1.0;
            end
            alpha = min(max(alpha, 0.0), 1.0);

            % Measured roll rate (rad/s)
            wx = state(obj.idx.omega_x);

            % Reference roll rate (rad/s) — adjust if you have a schedule/reference input
            wxRef = 0;

            % Error convention: e = w_desired - w_measured
            err = wxRef - wx;

            % Derivative on error
            derr = (err - obj.prevErr) / obj.dt;

            % Effective actuator torque limits after derating
            u_max =  alpha * obj.tauMax;
            u_min = -alpha * obj.tauMax;

            % I-term torque budget (fraction of available torque authority)
            I_max = obj.gamma * u_max;
            I_min = obj.gamma * u_min;

            % Pre-clamp stored I in case alpha shrank since last call
            obj.I_stored = min(max(obj.I_stored, I_min), I_max);

            % Candidate I update (I_stored is TORQUE units): I += Ki * e * dt
            I = min(max(obj.I_stored + (obj.Ki * err * obj.dt), I_min), I_max);

            % PID terms
            P = obj.Kp * err;
            D = obj.Kd * derr;

            % Unclamped command (matches C++ "unclamped")
            tauUnsat = P + I + D;

            % Clamp to actuator limits (matches C++ "clamped")
            tau = min(max(tauUnsat, u_min), u_max);

            % Anti-windup freeze conditions (matches C++)
            windup_high = (tauUnsat > u_max) && (err > 0.0);
            windup_low  = (tauUnsat < u_min) && (err < 0.0);

            if ~(windup_high || windup_low)
                obj.I_stored = I;
            end

            obj.prevErr = err;

            % Repo convention: 3x1 [roll; pitch; yaw]
            u = [tau; 0; 0];
        end

        function reset(obj)
            obj.I_stored = 0;
            obj.prevErr  = 0;
        end

        function populateBreakPoints(~)
            % No gain scheduling
        end
    end
end