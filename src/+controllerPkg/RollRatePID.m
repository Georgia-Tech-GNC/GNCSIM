% GNCSIM/src/+controllerPkg/RollRatePID.m
classdef RollRatePID < controllerPkg.Controller
    properties
        % Required by repo scheduling logic even if unused
        breakPoints
        useActuatorModel = true
        referenceAttitudes = []

        % Gains
        Kp
        Ki
        Kd

        % Integral budget fraction (0..1)
        gamma

        % Derivative low-pass filter coefficient (0..1)
        beta

        % State (paper semantics)
        xI      = 0      % integral state in "error-integral" units (NOT torque)
        prevErr = 0
        dErrF   = 0      % filtered derivative of error

        % Indices
        idx = StateIndices
    end

    methods
        function obj = RollRatePID(Kp, Ki, Kd, gamma, beta)
            obj.Kp = Kp; obj.Ki = Ki; obj.Kd = Kd;
            obj.gamma = gamma;

            if nargin < 6 || isempty(beta)
                beta = 0.8;
            end
            obj.beta = beta;

            % Repo scheduling logic expects this field even if unused.
            % Use nominal 100 Hz for breakpoint interval.
            obj.breakPoints = struct('cumVelocityInterval', 0.01);
        end

        function u = getControl(obj, state, ~, ~, dt_k, tau_max)
            % Thin wrapper: computes roll-rate error from state and delegates
            % to the paper-faithful PID step(err, dt_k, tau_max).
            %
            % Inputs:
            %   state   : rocket state vector (expects omega_x index)
            %   dt_k    : timestep for this control step (seconds)
            %   tau_max : allowable torque bound at this step (>0)
            %
            % Output:
            %   u : 3x1 torque command [tau_x; 0; 0]

            % Measured roll rate (rad/s)
            wx_meas = state(obj.idx.omega_x);

            % Desired roll rate (rad/s) — strict stabilization
            wx_des = 0;

            % Error: e = w_des - w_meas
            err = wx_des - wx_meas;

            tau_cmd = obj.step(err, dt_k, tau_max);

            % Repo convention: 3x1 [roll; pitch; yaw]
            u = [tau_cmd; 0; 0];
        end

        function tau_cmd = step(obj, err, dt_k, tau_max)
            % Paper-faithful PID step:
            % Inputs: err, dt_k, tau_max only
            % Output: tau_cmd

            % --- Derivative filtering (Eq. d_filter) ---
            dErrRaw = (err - obj.prevErr) / dt_k;
            obj.dErrF = obj.beta * obj.dErrF + (1 - obj.beta) * dErrRaw;

            % --- Integral budget (Eq. integral_budget) ---
            % tau_I_max = gamma * tau_max
            tau_I_max = obj.gamma * tau_max;

            % x_I_max = tau_I_max / Ki (handle Ki=0 safely)
            if obj.Ki > 0
                xI_max = tau_I_max / obj.Ki;
            else
                xI_max = 0.0;
            end

            % --- Pre-clamp stored integral to current bounds (Eq. integrator_preclamp) ---
            xI0 = obj.clamp(obj.xI, -xI_max, xI_max);

            % --- Position-form pre-clamp torque (Eq. pid_position_form) ---
            tau_pre = obj.Kp * err + obj.Ki * xI0 + obj.Kd * obj.dErrF;

            % --- Integral update + clamp (Eq. integrator_update, integrator_clamp) ---
            xI_star  = xI0 + err * dt_k;
            xI_clamp = obj.clamp(xI_star, -xI_max, xI_max);

            % --- Output clamp (Eq. tau_clamp) ---
            tau_cmd = obj.clamp(tau_pre, -tau_max, tau_max);

            % --- Conditional-integration anti-windup (Eq. anti_windup_freeze) ---
            windup_high = (tau_pre >  tau_max) && (err > 0);
            windup_low  = (tau_pre < -tau_max) && (err < 0);

            if windup_high || windup_low
                obj.xI = xI0;
            else
                obj.xI = xI_clamp;
            end

            % Update memory
            obj.prevErr = err;
        end

        function reset(obj)
            obj.xI = 0;
            obj.prevErr = 0;
            obj.dErrF = 0;
        end

        function populateBreakPoints(~)
            % No gain scheduling
        end
    end

    methods (Static, Access = private)
        function y = clamp(x, lo, hi)
            y = min(max(x, lo), hi);
        end
    end
end