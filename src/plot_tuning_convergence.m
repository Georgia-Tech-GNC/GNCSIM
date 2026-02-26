% plot_tuning_convergence.m
% Load tuning_out/bayesopt_pid.mat (saved by TunePID.m) and plot convergence.

clear; clc;

% Run from GNCSIM/src
addpath(genpath(pwd));

inFile = fullfile(pwd, "tuning_out", "bayesopt_pid.mat");
outDir = fullfile(pwd, "tuning_out", "plots");
outPng = fullfile(outDir, "tuning_convergence.png");

if ~isfile(inFile)
    error("plot_tuning_convergence:MissingFile", "Cannot find: %s", inFile);
end

S = load(inFile);

if ~isfield(S, "results")
    error("plot_tuning_convergence:MissingVar", ...
        "Expected variable `results` in %s (TunePID saves this).", inFile);
end

results = S.results;

% ---- Extract per-eval objective trace ----
if isprop(results, "ObjectiveTrace") && ~isempty(results.ObjectiveTrace)
    objTrace = results.ObjectiveTrace(:);
else
    error("plot_tuning_convergence:MissingObjectiveTrace", ...
        "results.ObjectiveTrace is missing/empty. Cannot plot convergence.");
end

% ---- Extract best-so-far trace (robust across MATLAB versions) ----
if isprop(results, "MinObjectiveTrace") && ~isempty(results.MinObjectiveTrace)
    bestTrace = results.MinObjectiveTrace(:);
else
    % Compute best-so-far from ObjectiveTrace
    bestTrace = nan(size(objTrace));
    best = inf;
    for k = 1:numel(objTrace)
        v = objTrace(k);
        if isfinite(v) && v < best
            best = v;
        end
        bestTrace(k) = best;
    end
end

n = min(numel(objTrace), numel(bestTrace));
objTrace  = objTrace(1:n);
bestTrace = bestTrace(1:n);
x = (1:n).';

if ~exist(outDir, "dir"), mkdir(outDir); end

fig = figure('Color','w'); grid on; hold on;
plot(x, bestTrace, 'LineWidth', 2);
plot(x, objTrace, 'LineWidth', 1);

xlabel("BO evaluation");
ylabel("Objective J(\theta)");
title("Bayesian Optimization Convergence (Syndrome PID)");
legend("Best-so-far J(\theta)", "Per-eval J(\theta)", "Location", "best");

set(gca, 'YScale', 'log'); % your J is huge (fail_cost), log is mandatory

finalBest = bestTrace(end);
text(0.02, 0.05, sprintf("final best = %.6g", finalBest), 'Units','normalized');

saveas(fig, outPng);
close(fig);

fprintf("Saved: %s\n", outPng);