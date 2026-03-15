function BuildSyndromePrecomputedAero()
% BuildSyndromePrecomputedAero
% Generate and save DATCOM-derived aero tables for the current Syndrome
% geometry/flight-condition deck so later runs can use precomputed_only mode.

    srcDir = fileparts(mfilename('fullpath'));
    projectRoot = fileparts(srcDir);
    paramsPath = fullfile(srcDir, "+SyndromeFiles", "params.json");
    precomputedDir = fullfile(projectRoot, "DATCOM", "precomputed");

    if ~isfile(paramsPath)
        error("Syndrome:Setup:ParamsFileNotFound", ...
            "Syndrome params file not found: %s", paramsPath);
    end

    params = jsondecode(fileread(paramsPath));

    if ~exist(precomputedDir, "dir")
        mkdir(precomputedDir);
    end

    opts = struct();
    opts.paramsPath = paramsPath;
    opts.aeroSourceMode = "runtime_cache";
    opts.precomputedAeroDir = precomputedDir;
    opts.setupOnly = true;

    % Force the current BODY/FIN decks to be generated and cached from the
    % exact same setup path used by TunePID warmup.
    [~, ~, rocketDynamics] = SyndromeRunOnce(params.controller.kp, params.controller.ki, params.controller.kd, opts);

    deckTexts = [
        string(rocketDynamics.body.lastDatcomDeckText)
        string(rocketDynamics.rocketFins.lastDatcomDeckText)
    ];
    savedFiles = strings(0, 1);

    for k = 1:numel(deckTexts)
        deckText = deckTexts(k);
        if strlength(deckText) == 0
            error("Syndrome:DATCOM:DeckNotGenerated", ...
                "Expected DATCOM deck text was not generated for Syndrome precompute target %d.", k);
        end

        [precomputedFile, cacheFile] = rocketPkg.DatcomManager.getAeroTargetsFromDeckText(deckText, precomputedDir);
        if ~isfile(cacheFile)
            error("Syndrome:DATCOM:CacheMissing", ...
                "Expected DATCOM cache file was not created for Syndrome precompute target %d: %s", ...
                k, cacheFile);
        end

        S = load(cacheFile, 'aero');
        rocketPkg.DatcomManager.savePrecomputedAero(precomputedFile, S.aero, deckText);
        savedFiles(end + 1, 1) = string(precomputedFile); %#ok<AGROW>
    end

    fprintf("Saved precomputed DATCOM aero tables:\n");
    for k = 1:numel(savedFiles)
        fprintf("  %s\n", savedFiles(k));
    end
end
