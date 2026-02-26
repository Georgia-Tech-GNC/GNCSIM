% plot_syndrome_run.m
% Load a saved Syndrome log (.mat produced by Log.saveToFile) and generate plots.

clear; clc;

% Ensure package folders are on path (run this from GNCSIM/src)
addpath(genpath(pwd));

% ---- User config ----
logFile = fullfile(pwd, "tuning_out", "syndrome_run_log.mat");  % change as needed
outDir  = fullfile(pwd, "tuning_out", "plots");                % where PNGs go
tStart  = [];   % e.g., 0
tEnd    = [];   % e.g., 10
prefix  = "run_"; % filename prefix

% ---- Load ----
data = analysisPkg.SyndromeAnalysis.loadSavedLog(logFile);

% ---- Plot ----
opts = struct();
opts.outDir = outDir;
opts.tStart = tStart;
opts.tEnd   = tEnd;
opts.prefix = prefix;

analysisPkg.SyndromeAnalysis.plotRun(data, opts);

fprintf("Saved plots to: %s\n", outDir);