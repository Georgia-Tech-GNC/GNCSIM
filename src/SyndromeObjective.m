function [l, detail] = SyndromeObjective(kp, ki, kd, weights, opts)
%SyndromeObjective  Run one GNCSIM Syndrome rollout with specified PID gains and
% compute scalar per-trial cost l per LaTeX spec:
%   ell_e = (1/T) * ∫ e(t)^2 dt
%   ell_u = (1/T) * ∫ tau_cmd(t)^2 dt
%   ell_r = (1/T) * ∫ (d/dt tau_cmd(t))^2 dt
%   l = w_e*ell_e + w_u*ell_u + w_r*ell_r
%
% tau_cmd(t) is the commanded body torque AFTER clamping to +/- tau_max(t),
% where tau_max(t) is computed by the sim and logged (derived from noisy
% ESC telemetry and motor voltage headroom limits).
%
% Discrete-time evaluation:
%   - integrals via trapezoidal rule (trapz)
%   - derivative via finite difference of successive torque commands

    if nargin < 4 || isempty(weights), error('weights struct required'); end
    if nargin < 5, opts = struct(); end
    if ~isfield(opts, 'fail_cost'), opts.fail_cost = 1e9; end

    req = {'w_e','w_u','w_r'};
    for k = 1:numel(req)
        if ~isfield(weights, req{k})
            error('weights.%s is required', req{k});
        end
    end

    % ---- Run sim ----
    try
        [log, params] = SyndromeRunOnce(kp, ki, kd, opts);
    catch ME
        print("Error on line 32");
        l = opts.fail_cost;
        detail = struct('failed', true, 'stage', 'SyndromeRunOnce', ...
                        'identifier', ME.identifier, 'message', ME.message);
        return;
    end

    % ---- Extract signals ----
    try
        t = log.time(:);
        if numel(t) < 3 || any(~isfinite(t)) || any(diff(t) <= 0)
            error('Bad time vector');
        end

        idx = StateIndices;
        wx = log.states(idx.omega_x, :).';          % Nx1
        tau_raw = log.controlInputs(1, :).';        % Nx1

        % Actuator logging vector (must match Flight.m actuatorStates logging):
        % 1: omega_w_true, 2: omega_w_meas, 3: V_bus_true, 4: V_bus_meas, 5: tau_max_meas
        omega_w_true = log.actuatorStates(1, :).';  %#ok<NASGU>
        tau_max_t    = log.actuatorStates(5, :).';  % Nx1, >= 0

        if any(~isfinite(wx)) || any(~isfinite(tau_raw)) || any(~isfinite(tau_max_t))
            error('Non-finite signals');
        end
        if any(tau_max_t < 0)
            error('Negative tau_max_t');
        end
    catch ME
        print("Error on line 61");
        l = opts.fail_cost;
        detail = struct('failed', true, 'stage', 'ExtractSignals', ...
                        'identifier', ME.identifier, 'message', ME.message);

        % Optional: save the log for inspection if requested
        if isfield(opts,'debug_fail_save') && opts.debug_fail_save && exist('log','var')
            try
                if ~isfield(opts,'debug_fail_dir') || isempty(opts.debug_fail_dir)
                    opts.debug_fail_dir = fullfile(pwd, "tuning_out", "fail_logs");
                end
                if ~exist(opts.debug_fail_dir, 'dir'), mkdir(opts.debug_fail_dir); end
                fname = sprintf("fail_kp%.3e_ki%.3e_kd%.3e.mat", kp, ki, kd);
                log.saveToFile(fullfile(opts.debug_fail_dir, fname));
            catch
                % ignore secondary failures
            end
        end
        return;
    end

    % ---- Desired roll-rate profile ----
    if isfield(opts, 'w_des_fn') && ~isempty(opts.w_des_fn)
        w_des = opts.w_des_fn(t, log, params);
        w_des = w_des(:);
        if numel(w_des) ~= numel(t) || any(~isfinite(w_des))
            print("Error on line 88");
            l = opts.fail_cost;
            detail = struct('failed', true);
            return;
        end
    else
        w_des = zeros(size(t));
    end

    % Error: e = w_des - w_meas
    e = w_des - wx;

    % ---- Clamp commanded torque ----
    tau_cmd = min(max(tau_raw, -tau_max_t), tau_max_t);

    % ---- Finite-difference derivative of commanded torque ----
    dt = diff(t);
    d_tau = diff(tau_cmd);
    tau_dot = [0; d_tau ./ dt];

    % ---- Averaged quadratic metrics ----
    Tspan = t(end) - t(1);
    if ~isfinite(Tspan) || Tspan <= 0
        print("Error on line 111");
        l = opts.fail_cost;
        detail = struct('failed', true);
        return;
    end

    ell_e = (1 / Tspan) * trapz(t, e.^2);
    ell_u = (1 / Tspan) * trapz(t, tau_cmd.^2);
    ell_r = (1 / Tspan) * trapz(t, tau_dot.^2);

    if any(~isfinite([ell_e ell_u ell_r])) || any([ell_e ell_u ell_r] < 0)
        print("NOT FINITE");
        l = opts.fail_cost;
        detail = struct('failed', true);
        return;
    end

    % ---- Total per-trial cost ----
    l = weights.w_e * ell_e + weights.w_u * ell_u + weights.w_r * ell_r;

    % ---- Detail ----
    detail = struct();
    detail.failed = false;

    detail.ell_e = ell_e;
    detail.ell_u = ell_u;
    detail.ell_r = ell_r;
    detail.l = l;

    detail.t = t;
    detail.Tspan = Tspan;

  
    detail.omega_w_true = log.actuatorStates(1,:).';
    detail.omega_w_meas = log.actuatorStates(2,:).';
    detail.V_bus_meas = log.actuatorStates(4,:).';
    
    detail.e = e;
    detail.w_des = w_des;
    detail.wx = wx;

    detail.tau_raw = tau_raw;
    detail.tau_cmd = tau_cmd;
    detail.tau_dot = tau_dot;

    detail.tau_max_t = tau_max_t;
end