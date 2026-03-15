scriptDir = fileparts(mfilename('fullpath'));

opts = struct();
opts.paramsPath = fullfile(scriptDir, "+SyndromeFiles", "params.json");
opts.fail_cost  = 1e9;
opts.scenario   = struct(); % nominal

try
    [log, params] = SyndromeRunOnce(1e-2, 1e-3, 1e-2, opts);
    disp("SyndromeRunOnce OK");
catch ME
    fprintf(2, "SyndromeRunOnce FAIL: %s\n", ME.message);
    for k = 1:numel(ME.stack)
        fprintf(2, "  at %s:%d\n", ME.stack(k).file, ME.stack(k).line);
    end
end
