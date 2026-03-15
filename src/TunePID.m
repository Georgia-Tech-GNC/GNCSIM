function results = TunePID()
% TunePID  Bayesian optimization driver for Syndrome PID gains (stochastic MC objective).
%
% Per paper:
%   Per-trial cost:            l(theta; s)  from SyndromeObjective()
%   Aggregated MC objective:   J(theta) = mean(l_i) + lambda * P95({l_i})
%   Optimization variables:    (log10 Kp, log10 Ki, log10 Kd)
%   Objective determinism:     stochastic (fresh scenario draws each trial)
%   Stopping:                 early-stop on convergence (with a safety cap)

    srcDir = localSrcDir();
    outDir = fullfile(srcDir, "tuning_out");

    %% Options passed through objective -> SyndromeRunOnce
    opts = struct();
    opts.paramsPath = fullfile(srcDir, "+SyndromeFiles", "params.json");
    opts.fail_cost  = 1e9;
    opts.debug_fail_save = true;
    opts.debug_fail_dir  = fullfile(outDir, "fail_logs");
    opts.aeroSourceMode = "precomputed_only";
    opts.precomputedAeroDir = fullfile(fileparts(srcDir), "DATCOM", "precomputed");

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

        if useParallelMC
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
                catch ME
                    if isSetupFailure(ME)
                        rethrow(ME);
                    end
                    li(i) = opts.fail_cost;
                    failed(i) = true;
                end
            end
        else
            for i = 1:mc.N
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
                catch ME
                    if isSetupFailure(ME)
                        rethrow(ME);
                    end
                    li(i) = opts.fail_cost;
                    failed(i) = true;
                end
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

        % Get best-so-far trace robustly across MATLAB versions
        trace = [];

        if isprop(boResults, "ObjectiveMinimumTrace")
            trace = boResults.ObjectiveMinimumTrace;
        elseif isprop(boResults, "ObjectiveTrace")
            ot = boResults.ObjectiveTrace;
            trace = cummin(ot); % best-so-far from per-eval objective
        elseif isprop(boResults, "MinObjective") && isprop(boResults, "NumObjectiveEvaluations")
            % Last-resort: cannot form a windowed trace -> disable early-stop
            return;
        else
            return;
        end

        trace = trace(:);
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
        optsWarm.setupOnly = true;
        SyndromeRunOnce(1, 0, 0, optsWarm); % cheap dummy
    catch ME
        error("Syndrome:Tuning:SetupFailed", ...
            "TunePID setup failed during DATCOM/aero warmup: %s (%s)", ...
            ME.message, ME.identifier);
    end

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

    %% Run Bayesian optimization
    results = bayesopt(@objFcn, vars, ...
        'MaxObjectiveEvaluations', maxEvalsCap, ...
        'NumSeedPoints', nSeed, ...
        'IsObjectiveDeterministic', false, ...
        'UseParallel', useParallelBO, ...
        'AcquisitionFunctionName', 'expected-improvement-plus', ...
        'OutputFcn', @stopFcn, ...
        'PlotFcn', {}, ...
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
        bestTrace = results.ObjectiveMinimumTrace(:);
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

function tf = isSetupFailure(ME)
    tf = startsWith(string(ME.identifier), "Syndrome:Setup:") || ...
         startsWith(string(ME.identifier), "Syndrome:DATCOM:");
end

function srcDir = localSrcDir()
    thisFile = mfilename('fullpath');
    srcDir = fileparts(thisFile);
end

%% Fresh scenario draw per rollout (stochastic; no CRN)
function s = drawScenario_rollrate()
    s = struct();

    % Plant
    % Use a tighter bounded MOI variation for tuning. The paper's early
    % uncertainty table is very loose for flight-tuning purposes; until a
    % measured inertia range is available, keep the MC spread to +/-10%.
    moiRelTol = 0.10;
    s.Ix_scale = boundedUniformScale(moiRelTol);

    % The paper identifies c_d as a tuned/identified effective damping term but
    % does not provide a measurement-based tolerance yet. Keep this bounded and
    % modest until hardware characterization replaces it.
    s.rollDampLin_scale = boundedUniformScale(0.15);

    % Actuator
    s.Iw_scale = boundedUniformScale(0.05);

    % Friction parameters are not yet hardware-identified in this model, and
    % the nominal viscous friction is currently zero. Avoid inventing a large
    % spread here until measured values are available.
    s.b_scale = 1.0;

    s.Imax_scale = boundedUniformScale(0.05);
    s.Vbus_scale = boundedUniformScale(0.02);

    % Initial condition
    s.wx0 = boundedUniform(-deg2rad(10), deg2rad(10));

    % Gyro (x-axis dominates roll-rate control)
    s.gyro_bias_x = boundedUniform(-deg2rad(0.2), deg2rad(0.2));
    s.gyro_biasRW_scale = boundedUniformScale(0.10);
    s.gyro_noise_scale  = boundedUniformScale(0.10);
    s.gyro_latency_s    = boundedUniform(0.009, 0.011);

    % Scheduler jitter (FreeRTOS-style)
    s.p_miss1 = boundedUniform(0.03, 0.07);
    s.p_miss2 = boundedUniform(0.002, 0.008);
    s.latency_max_s = boundedUniform(5e-5, 1.5e-4);

    % External roll disturbance torque: paper-model stress case
    % tau_dist(t) = tau_bias + tau_step * 1[t >= t0]
    % Keep these bounded around the nominal stress case instead of allowing
    % long-tail draws to dominate the objective.
    tauBiasMag = boundedUniform(0.5e-3, 1.5e-3);
    tauStepMag = boundedUniform(1.0e-3, 3.0e-3);
    s.tau_dist_bias_x = tauBiasMag * sign(randn());
    s.tau_dist_step_x = tauStepMag * sign(randn());
    s.tau_dist_step_t0_s = boundedUniform(0.20, 0.80);
end

function x = boundedUniform(lo, hi)
    x = lo + (hi - lo) * rand();
end

function scale = boundedUniformScale(relTol)
    scale = boundedUniform(1 - relTol, 1 + relTol);
end
