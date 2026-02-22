function results = TunePID()
% TunePID  Bayesian optimization driver for Syndrome PID gains (stochastic MC objective).
%
% Uses Monte Carlo rollouts inside the objective. Each bayesopt evaluation
% draws fresh scenarios (non-CRN), so the objective is stochastic.
%
% Assumes on MATLAB path:
%   - SyndromeRunOnce.m   : [log, params] = SyndromeRunOnce(kp,ki,kd,opts)
%   - SyndromeObjective.m : [J, detail] = SyndromeObjective(kp,ki,kd,weights,opts)

    %% Options passed through objective -> SyndromeRunOnce
    opts = struct();
    opts.paramsPath = fullfile(pwd, "+SyndromeFiles", "params.json");
    opts.fail_cost  = 1e9;

    %% Cost weights
    weights = struct();
    weights.w_e = 1.0;
    weights.w_u = 1e-4;
    weights.w_s = 10.0;
    weights.w_r = 1e-6;
    weights.eps_frac = 0.01;
    weights.eps_abs  = 0.0;

    %% Monte Carlo aggregation settings (stochastic)
    mc = struct();
    mc.N = 12;              % rollouts per BO evaluation
    mc.lambdaP95 = 0.5;     % weight on tail risk
    mc.failPenalty = 1e4;   % added per failed rollout

    %% BO budget
    maxEvals = 120;
    nSeed    = 20;
    useParallel = false;

    %% Output
    outDir = fullfile(pwd, "tuning_out");
    if ~exist(outDir, "dir"), mkdir(outDir); end
    saveFile = fullfile(outDir, "bayesopt_pid.mat");

    %% Decision variables (log space)
    vars = [
        optimizableVariable('logKp', [-3, 3],  'Type','real')  % Kp in [1e-3, 1e3]
        optimizableVariable('logKi', [-6, 2],  'Type','real')  % Ki in [1e-6, 1e2]
        optimizableVariable('logKd', [-6, 2],  'Type','real')  % Kd in [1e-6, 1e2]
    ];

    %% Objective wrapper (stochastic MC)
    function J = objFcn(X)
        kp = 10.^X.logKp;
        ki = 10.^X.logKi;
        kd = 10.^X.logKd;

        % Sanity
        if ~isfinite(kp) || ~isfinite(ki) || ~isfinite(kd) || kp <= 0 || ki < 0 || kd < 0
            J = opts.fail_cost;
            return;
        end

        Ji = zeros(mc.N,1);
        failed = false(mc.N,1);

        for i = 1:mc.N
            opts_i = opts;
            opts_i.scenario = drawScenario_rollrate();

            try
                [Ji(i), detail] = SyndromeObjective(kp, ki, kd, weights, opts_i);

                if ~isfinite(Ji(i)) || Ji(i) < 0
                    Ji(i) = opts.fail_cost;
                    failed(i) = true;
                elseif isfield(detail,'failed') && detail.failed
                    failed(i) = true;
                end
            catch
                Ji(i) = opts.fail_cost;
                failed(i) = true;
            end
        end

        Jmean  = mean(Ji);
        Jp95   = prctile(Ji, 95);
        nfails = sum(failed);

        J = Jmean + mc.lambdaP95 * Jp95 + mc.failPenalty * nfails;

        if ~isfinite(J) || J < 0
            J = opts.fail_cost;
        end
    end

    %% Run Bayesian optimization (stochastic objective)
    results = bayesopt(@objFcn, vars, ...
        'MaxObjectiveEvaluations', maxEvals, ...
        'NumSeedPoints', nSeed, ...
        'IsObjectiveDeterministic', false, ...
        'UseParallel', useParallel, ...
        'AcquisitionFunctionName', 'expected-improvement-plus', ...
        'Verbose', 1);

    %% Save best + metadata
    bestX = results.XAtMinObjective;
    best = struct();
    best.Kp = 10.^bestX.logKp;
    best.Ki = 10.^bestX.logKi;
    best.Kd = 10.^bestX.logKd;
    best.J  = results.MinObjective;

    save(saveFile, "results", "best", "weights", "opts", "mc");

    fprintf("\nBest:\n  Kp=%.6e\n  Ki=%.6e\n  Kd=%.6e\n  J =%.6g\nSaved: %s\n", ...
        best.Kp, best.Ki, best.Kd, best.J, saveFile);
end

%% Fresh scenario draw per rollout (stochastic; no CRN)
function s = drawScenario_rollrate()
    s = struct();

    % Plant
    s.Ix_scale = 1 + 0.20*randn();
    s.rollDampLin_scale = exp(0.25*randn());

    % Actuator
    s.tauMax_scale = exp(0.10*randn());
    s.Iw_scale     = exp(0.10*randn());
    s.b_scale      = exp(0.30*randn());

    % Initial condition
    s.wx0 = deg2rad(30) * (2*rand() - 1);

    % Gyro (x-axis dominates roll-rate control)
    s.gyro_bias_x = deg2rad(0.5) * randn();  % rad/s
    s.gyro_biasRW_scale = exp(0.30*randn());
    s.gyro_noise_scale  = exp(0.30*randn());
    s.gyro_latency_s    = max(0, 0.010 + 0.002*randn());
end