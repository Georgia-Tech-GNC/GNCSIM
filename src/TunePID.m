function results = TunePID()
% TunePID  Bayesian optimization driver for Syndrome PID gains (stochastic MC objective).
%
% Per paper:
%   Per-trial cost:            l(theta; s)  from SyndromeObjective()
%   Aggregated MC objective:   J(theta) = mean(l_i) + lambda * P95({l_i})
%   Optimization variables:    (log10 Kp, log10 Ki, log10 Kd)
%   Objective determinism:     stochastic (fresh scenario draws each trial)
%   Stopping:                 early-stop on convergence (with a safety cap)

    %% Options passed through objective -> SyndromeRunOnce
    opts = struct();
    opts.paramsPath = fullfile(pwd, "+SyndromeFiles", "params.json");
    opts.fail_cost  = 1e9;

    %% Per-trial objective weights (LaTeX: (w_e, w_u, w_r))
    weights = struct();
    weights.w_e = 1.0;
    weights.w_u = 1e-4;
    weights.w_r = 1e-6;

    %% Monte Carlo aggregation settings (LaTeX)
    mc = struct();
    mc.N      = 12;   % trials per BO evaluation (set to your paper's N once decided)
    mc.lambda = 0.5;  % tail-risk weight on P95

    useParallelBO = false;
    useParallelMC = true;

    if useParallelMC
        p = gcp('nocreate');
        if isempty(p)
            try
                parpool('threads');
            catch
                parpool('local'); % fallback
            end
        end
    end
    % Early-stopping (convergence) controls
    conv = struct();
    conv.minEvals   = 30;      % don't stop too early
    conv.patience   = 25;      % window length with no meaningful improvement
    conv.relTol     = 1e-3;    % relative improvement threshold (0.1%)
    conv.absTol     = 1e-6;    % absolute improvement floor

    % Safety cap (still required to prevent infinite runs)
    maxEvalsCap = 500;
    nSeed       = 20;

    %% Output
    outDir = fullfile(pwd, "tuning_out");
    if ~exist(outDir, "dir"), mkdir(outDir); end
    saveFile = fullfile(outDir, "bayesopt_pid.mat");

    %% Decision variables (log space)
    vars = [
        optimizableVariable('logKp', [-3, 3], 'Type','real')  % Kp in [1e-3, 1e3]
        optimizableVariable('logKi', [-6, 2], 'Type','real')  % Ki in [1e-6, 1e2]
        optimizableVariable('logKd', [-6, 2], 'Type','real')  % Kd in [1e-6, 1e2]
    ];

    %% Objective wrapper: returns aggregated J(theta)
    function J = objFcn(X)
        kp = 10.^X.logKp;
        ki = 10.^X.logKi;
        kd = 10.^X.logKd;

        % Sanity
        if ~isfinite(kp) || ~isfinite(ki) || ~isfinite(kd) || kp <= 0 || ki < 0 || kd < 0
            J = opts.fail_cost;
            return;
        end

        li = zeros(mc.N, 1);       % per-trial costs l(theta; s_i)
        failed = false(mc.N, 1);

        parfor i = 1:mc.N
            opts_i = opts;
            opts_i.scenario = drawScenario_rollrate(); % fresh draw => stochastic objective

            try
                [li(i), detail] = SyndromeObjective(kp, ki, kd, weights, opts_i);

                if ~isfinite(li(i)) || li(i) < 0
                    li(i) = opts.fail_cost;
                    failed(i) = true;
                elseif isfield(detail,'failed') && detail.failed
                    li(i) = opts.fail_cost;
                    failed(i) = true;
                end
            catch
                li(i) = opts.fail_cost;
                failed(i) = true;
            end
        end

        % Per LaTeX: J = mean(l_i) + lambda*P95(l_i)
        Jmean = mean(li);
        Jp95  = prctile(li, 95);
        J     = Jmean + mc.lambda * Jp95;

        % Failure handling (paper: "large penalty for failed trials"):
        % already enforced by setting li(i)=fail_cost, which increases mean and P95.
        if ~isfinite(J) || J < 0
            J = opts.fail_cost;
        end
    end

    %% Convergence stop function (early stopping)
    function stop = stopFcn(boResults, state)
        %#ok<INUSD>
        stop = false;

        % Only meaningful after an iteration completes
        if ~strcmp(state, "iteration")
            return;
        end

        % Need enough completed evaluations
        n = boResults.NumObjectiveEvaluations;
        if n < conv.minEvals
            return;
        end

        % If not enough history to check patience window, do nothing
        if n < conv.patience + 1
            return;
        end

        % Best objective so far at each iteration is in MinObjectiveTrace
        trace = boResults.MinObjectiveTrace;
        if numel(trace) < conv.patience + 1 || any(~isfinite(trace))
            return;
        end

        best_now    = trace(end);
        best_before = trace(end - conv.patience);

        % Improvement is decrease in objective
        abs_impr = best_before - best_now;
        rel_impr = abs_impr / max(abs(best_before), eps);

        % Converged if improvement over the patience window is tiny
        if (abs_impr <= conv.absTol) || (rel_impr <= conv.relTol)
            stop = true;
        end
    end

    % Pre-warm DATCOM cache once (any gains are fine)
    try
        optsWarm = opts;
        optsWarm.scenario = struct(); % no overrides
        SyndromeRunOnce(1, 0, 0, optsWarm); % cheap dummy
    catch
        % ignore warm-up failures; BO will surface real errors
    end

    %% Run Bayesian optimization
    results = bayesopt(@objFcn, vars, ...
        'MaxObjectiveEvaluations', maxEvalsCap, ...
        'NumSeedPoints', nSeed, ...
        'IsObjectiveDeterministic', false, ...
        'UseParallel', useParallelBO, ...
        'AcquisitionFunctionName', 'expected-improvement-plus', ...
        'OutputFcn', @stopFcn, ...
        'Verbose', 1);

    %% Save best + metadata
    bestX = results.XAtMinObjective;
    best = struct();
    best.Kp = 10.^bestX.logKp;
    best.Ki = 10.^bestX.logKi;
    best.Kd = 10.^bestX.logKd;
    best.J  = results.MinObjective; % aggregated J(theta)

    save(saveFile, "results", "best", "weights", "opts", "mc", "conv", "maxEvalsCap", "nSeed");

    fprintf("\nBest (aggregated objective J):\n  Kp=%.6e\n  Ki=%.6e\n  Kd=%.6e\n  J =%.6g\nSaved: %s\n", ...
        best.Kp, best.Ki, best.Kd, best.J, saveFile);

    %% ----------------------------
    % Auto-generate plots + a saved "best run" log
    % ----------------------------
    plotsDir = fullfile(outDir, "plots");
    if ~exist(plotsDir, "dir"), mkdir(plotsDir); end

    % ---- (1) Convergence plot (best-so-far J vs eval #) ----
    try
        bestTrace = results.MinObjectiveTrace(:);
        x = (1:numel(bestTrace)).';

        fig = figure('Color','w');
        grid on; hold on;
        plot(x, bestTrace, 'LineWidth', 2);
        xlabel("BO evaluation");
        ylabel("Best-so-far J(\theta)");
        title("Bayesian Optimization Convergence (Syndrome PID)");
        set(gca, 'YScale', 'log');

        finalBest = bestTrace(end);
        text(0.02, 0.05, sprintf("final best = %.6g", finalBest), 'Units','normalized');

        saveas(fig, fullfile(plotsDir, "tuning_convergence.png"));
        close(fig);
    catch ME
        warning("TunePID:PlotConvergenceFailed", "Failed to generate convergence plot: %s", ME.message);
    end

    % ---- (2) Run one nominal rollout with best gains, save log, and plot ----
    try
        optsBest = opts;
        optsBest.scenario = struct(); % nominal (no MC overrides)

        [logBest, ~] = SyndromeRunOnce(best.Kp, best.Ki, best.Kd, optsBest);

        % Save for later (plot_syndrome_run.m can load this)
        bestLogFile = fullfile(outDir, "best_run_log.mat");
        logBest.saveToFile(bestLogFile);

        % Generate PNGs now
        plotOpts = struct();
        plotOpts.outDir = plotsDir;
        plotOpts.prefix = "best_";
        analysisPkg.SyndromeAnalysis.plotRun(logBest, plotOpts);

        fprintf("Saved best-run log: %s\n", bestLogFile);
        fprintf("Saved best-run plots to: %s\n", plotsDir);
    catch ME
        warning("TunePID:PlotBestRunFailed", "Failed to generate best-run plots/log: %s", ME.message);
    end
end

%% Fresh scenario draw per rollout (stochastic; no CRN)
function s = drawScenario_rollrate()
    s = struct();

    % Plant
    s.Ix_scale = 1 + 0.20*randn();
    s.rollDampLin_scale = exp(0.25*randn());

    % Actuator
    s.Iw_scale = exp(0.10*randn());
    s.b_scale  = exp(0.30*randn());

    s.Imax_scale = exp(0.10*randn());
    s.Vbus_scale = exp(0.03*randn());

    % Initial condition
    s.wx0 = deg2rad(30) * (2*rand() - 1);

    % Gyro (x-axis dominates roll-rate control)
    s.gyro_bias_x = deg2rad(0.5) * randn();  % rad/s
    s.gyro_biasRW_scale = exp(0.30*randn());
    s.gyro_noise_scale  = exp(0.30*randn());
    s.gyro_latency_s    = max(0, 0.010 + 0.002*randn());

    % Scheduler jitter (FreeRTOS-style)
    s.p_miss1 = max(0, 0.05 + 0.02*randn());
    s.p_miss2 = max(0, 0.005 + 0.003*randn());
    s.latency_max_s = max(0, 1e-4 + 5e-5*randn());
end