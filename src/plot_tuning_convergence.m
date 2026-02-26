% plot_tuning_convergence.m
%
% Based on GNCSIM/src/TunePID.m in the provided GNCSIM.zip:
% - TunePID saves: tuning_out/bayesopt_pid.mat containing variable `results`
% - `results` is a bayesopt BayesianOptimization object
% - Convergence is tracked in results.MinObjectiveTrace (best-so-far J)
%
% This script:
%   - loads `results`
%   - plots best-so-far objective vs evaluation
%   - optionally overlays raw ObjectiveTrace (if present)
%   - saves a PNG to tuning_out/plots/

clear; clc;

% Run from GNCSIM/src
addpath(genpath(pwd));

% --------------------
% Paths
% --------------------
inFile = fullfile(pwd, "tuning_out", "bayesopt_pid.mat");
outDir = fullfile(pwd, "tuning_out", "plots");
outPng = fullfile(outDir, "tuning_convergence.png");

if ~isfile(inFile)
    error("plot_tuning_convergence:MissingFile", "Cannot find: %s", inFile);
end

S = load(inFile);

if ~isfield(S, "results")
    error("plot_tuning_convergence:MissingVar", ...
        "Expected variable `results` in %s (saved by TunePID.m).", inFile);
end

results = S.results;

% --------------------
% Extract traces
% --------------------
if ~isprop(results, "MinObjectiveTrace") || isempty(results.MinObjectiveTrace)
    error("plot_tuning_convergence:MissingTrace", "results.MinObjectiveTrace is missing/empty.");
end

bestTrace = results.MinObjectiveTrace(:);
n = numel(bestTrace);
x = (1:n).';

haveObjTrace = isprop(results, "ObjectiveTrace") && ~isempty(results.ObjectiveTrace);
if haveObjTrace
    objTrace = results.ObjectiveTrace(:);
    % bayesopt sometimes stores extra entries; clamp to min length
    m = min(numel(objTrace), n);
    objTrace = objTrace(1:m);
else
    objTrace = [];
end

% --------------------
% Plot
% --------------------
if ~exist(outDir, "dir"), mkdir(outDir); end

fig = figure('Color', 'w');
grid on; hold on;

% Best-so-far
plot(x, bestTrace, 'LineWidth', 2);

% Raw objective (optional, lightly)
if ~isempty(objTrace)
    plot((1:numel(objTrace)).', objTrace, 'LineWidth', 1);
    legend("Best-so-far J(\theta)", "Per-eval J(\theta)", "Location", "best");
else
    legend("Best-so-far J(\theta)", "Location", "best");
end

xlabel("BO evaluation");
ylabel("Objective J(\theta)");
title("Bayesian Optimization Convergence (Syndrome PID)");

% Log scale usually helps because J can be heavy-tailed from fail_cost
set(gca, 'YScale', 'log');

% Annotate final best
finalBest = bestTrace(end);
txt = sprintf("final best = %.6g", finalBest);
text(0.02, 0.05, txt, 'Units', 'normalized');

saveas(fig, outPng);
close(fig);

fprintf("Saved: %s\n", outPng);