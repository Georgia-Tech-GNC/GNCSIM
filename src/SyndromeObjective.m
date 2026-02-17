function [J, detail] = SyndromeObjective(kp, ki, kd, weights, opts)
%SyndromeObjective  Run one GNCSIM Syndrome rollout with specified PID gains and
%                   compute scalar cost J.
%
% Requires the wrapper function:
%   [log, params] = SyndromeRunOnce(kp, ki, kd, opts)
%
% Inputs
%   kp, ki, kd : PID gains (scalars)
%   weights    : struct with fields:
%                 - w_e, w_u, w_s, w_r  (required)
%                 - eps_frac (optional, default 0.01)  % saturation margin as fraction of u_max
%                 - eps_abs  (optional, default 0.0)   % absolute saturation margin
%   opts       : struct (optional). Common fields:
%                 - paramsPath (optional) : path to params.json
%                 - w_des_fn (optional)   : function handle w_des = f(t, log, params)
%                 - fail_cost (optional)  : default 1e9
%
% Outputs
%   J      : scalar cost
%   detail : struct with components and key timeseries (useful for debugging)

    if nargin < 4 || isempty(weights), error('weights struct required'); end
    if nargin < 5, opts = struct(); end

    % Defaults
    if ~isfield(weights, 'eps_frac'), weights.eps_frac = 0.01; end
    if ~isfield(weights, 'eps_abs'),  weights.eps_abs  = 0.0;  end
    if ~isfield(opts, 'fail_cost'),   opts.fail_cost   = 1e9;  end

    % ---- Run sim (you supply SyndromeRunOnce) ----
    try
        [log, params] = SyndromeRunOnce(kp, ki, kd, opts);
    catch
        J = opts.fail_cost;
        detail = struct('failed', true);
        return;
    end

    % ---- Extract required signals ----
    try
        t = log.time(:); % Nx1
        if numel(t) < 3 || any(~isfinite(t)) || any(diff(t) <= 0)
            error('Bad time vector');
        end

        idx = StateIndices;
        wx = log.states(idx.omega_x, :).'; % Nx1

        % Control input: roll torque command (first element of [tau;0;0])
        u = log.controlInputs(1, :).'; % Nx1

        % Actuator state: wheel speed assumed actuatorStates(1,:) (matches SyndromeActuatorModel)
        w = log.actuatorStates(1, :).'; % Nx1

        if any(~isfinite(wx)) || any(~isfinite(u)) || any(~isfinite(w))
            error('Non-finite signals');
        end
    catch
        J = opts.fail_cost;
        detail = struct('failed', true);
        return;
    end

    % ---- Determine actuator limits / derating law params ----
    % tauMax: prefer controller.tau_max, else reaction_wheel.tau_max
    tauMax = [];
    if isfield(params, 'controller') && isfield(params.controller, 'tau_max')
        tauMax = params.controller.tau_max;
    elseif isfield(params, 'reaction_wheel') && isfield(params.reaction_wheel, 'tau_max')
        tauMax = params.reaction_wheel.tau_max;
    end
    if isempty(tauMax) || ~isfinite(tauMax) || tauMax <= 0
        J = opts.fail_cost;
        detail = struct('failed', true);
        return;
    end

    % wMax and wSoft: if w_soft missing, fall back to 0.75*w_max (conservative default)
    if isfield(params, 'reaction_wheel') && isfield(params.reaction_wheel, 'w_max')
        wMax = params.reaction_wheel.w_max;
    else
        J = opts.fail_cost;
        detail = struct('failed', true);
        return;
    end
    if isfield(params.reaction_wheel, 'w_soft')
        wSoft = params.reaction_wheel.w_soft;
    else
        wSoft = 0.75 * wMax;
    end
    if ~isfinite(wMax) || wMax <= 0 || ~isfinite(wSoft) || wSoft < 0 || wSoft >= wMax
        J = opts.fail_cost;
        detail = struct('failed', true);
        return;
    end

    % ---- Desired roll-rate profile ----
    if isfield(opts, 'w_des_fn') && ~isempty(opts.w_des_fn)
        w_des = opts.w_des_fn(t, log, params);
        w_des = w_des(:);
        if numel(w_des) ~= numel(t) || any(~isfinite(w_des))
            J = opts.fail_cost;
            detail = struct('failed', true);
            return;
        end
    else
        w_des = zeros(size(t)); % no-roll default
    end

    % Error: e = w_des - w_meas
    e = w_des - wx;

    % ---- Compute alpha(t) and u_max(t) ----
    absw = abs(w);
    alpha = ones(size(absw));
    alpha(absw >= wMax) = 0.0;

    mid = (absw > wSoft) & (absw < wMax);
    alpha(mid) = (wMax - absw(mid)) ./ (wMax - wSoft);

    % Clamp numeric safety
    alpha = min(max(alpha, 0.0), 1.0);

    u_max = alpha * tauMax; % Nx1, >=0

    % ---- du/dt ----
    dt = diff(t);
    du = diff(u);
    udot = [0; du ./ dt]; % Nx1

    % ---- Saturation indicator ----
    eps_margin = max(weights.eps_abs, weights.eps_frac .* u_max);
    % If u_max==0, eps_margin==0; saturation then means |u|>=0 (always true) so clamp:
    % we treat alpha==0 as "no authority", and saturating is only meaningful if u_max>0.
    sat = false(size(u));
    hasAuth = (u_max > 0);
    sat(hasAuth) = abs(u(hasAuth)) >= (u_max(hasAuth) - eps_margin(hasAuth));

    % ---- Integrals ----
    Je = trapz(t, e.^2);
    Ju = trapz(t, u.^2);
    Js = trapz(t, double(sat));
    Jr = trapz(t, udot.^2);

    if any(~isfinite([Je Ju Js Jr])) || any([Je Ju Js Jr] < 0)
        J = opts.fail_cost;
        detail = struct('failed', true);
        return;
    end

    % ---- Total cost ----
    J = weights.w_e * Je + weights.w_u * Ju + weights.w_s * Js + weights.w_r * Jr;

    % Optional details for debugging/plots
    detail = struct();
    detail.failed = false;
    detail.Je = Je; detail.Ju = Ju; detail.Js = Js; detail.Jr = Jr;
    detail.t = t; detail.e = e; detail.u = u; detail.udot = udot;
    detail.alpha = alpha; detail.u_max = u_max;
    detail.w = w; detail.wx = wx; detail.w_des = w_des;
    detail.tauMax = tauMax; detail.wSoft = wSoft; detail.wMax = wMax;
end