function results = TunePID()
%TunePID  Bayesian optimization driver for Syndrome PID gains.
%
% Assumes these are on the MATLAB path:
%   - SyndromeRunOnce.m   : [log, params] = SyndromeRunOnce(kp,ki,kd,opts)
%   - SyndromeObjective.m : [J, detail] = SyndromeObjective(kp,ki,kd,weights,opts)
%
% Returns:
%   results : bayesopt results object

    %% Options
    opts = struct();
    opts.paramsPath = fullfile(pwd, "+SyndromeFiles", "params.json");
    opts.fail_cost  = 1e9;

    % Cost weights
    weights = struct();
    weights.w_e = 1.0;
    weights.w_u = 1e-4;
    weights.w_s = 10.0;
    weights.w_r = 1e-6;
    weights.eps_frac = 0.01;   % saturation margin as fraction of u_max
    weights.eps_abs  = 0.0;    % absolute saturation margin

    % BO budget
    maxEvals = 120;            % total trials
    nSeed    = 20;             % initial random points

    % Parallelization (safe default: off)
    useParallel = false;

    % Save path
    outDir = fullfile(pwd, "tuning_out");
    if ~exist(outDir, "dir"), mkdir(outDir); end
    saveFile = fullfile(outDir, "bayesopt_pid.mat");

    %% Decision variables (log space)
    vars = [
        optimizableVariable('logKp', [-3, 3],  'Type','real')  % Kp in [1e-3, 1e3]
        optimizableVariable('logKi', [-6, 2],  'Type','real')  % Ki in [1e-6, 1e2]
        optimizableVariable('logKd', [-6, 2],  'Type','real')  % Kd in [1e-6, 1e2]
    ];

    %% Objective wrapper for bayesopt
    function J = objFcn(X)
        kp = 10.^X.logKp;
        ki = 10.^X.logKi;
        kd = 10.^X.logKd;

        % Basic sanity
        if ~isfinite(kp) || ~isfinite(ki) || ~isfinite(kd) || kp <= 0 || ki < 0 || kd < 0
            J = opts.fail_cost;
            return;
        end

        try
            J = SyndromeObjective(kp, ki, kd, weights, opts);
            if ~isfinite(J) || J < 0
                J = opts.fail_cost;
            end
        catch
            J = opts.fail_cost;
        end

        % print progress (comment out for silence)
        % fprintf("Kp=%.3e Ki=%.3e Kd=%.3e -> J=%.6g\n", kp, ki, kd, J);
    end

    %% Run Bayesian optimization
    results = bayesopt(@objFcn, vars, ...
        'MaxObjectiveEvaluations', maxEvals, ...
        'NumSeedPoints', nSeed, ...
        'IsObjectiveDeterministic', true, ...
        'UseParallel', useParallel, ...
        'AcquisitionFunctionName', 'expected-improvement-plus', ...
        'Verbose', 1);

    %% Convenience table of all trials (inputs + objective)
    T = results.XTrace;                 % table: logKp/logKi/logKd for each eval
    T.J = results.ObjectiveTrace(:);    % objective value for each eval

    %% Extract best gains and save
    bestX = results.XAtMinObjective;
    best = struct();
    best.Kp = 10.^bestX.logKp;
    best.Ki = 10.^bestX.logKi;
    best.Kd = 10.^bestX.logKd;
    best.J  = results.MinObjective;

    save(saveFile, "results", "best", "weights", "opts");

    fprintf("\nBest:\n  Kp=%.6e\n  Ki=%.6e\n  Kd=%.6e\n  J =%.6g\nSaved: %s\n", ...
        best.Kp, best.Ki, best.Kd, best.J, saveFile);
end