function S = buildCfdAeroMat(xlsxFile, outFile)
% Vibe COded

    arguments
        xlsxFile (1,1) string
        outFile (1,1) string = "cfd_aero.mat"
    end

    rho = 1.225;   

    [alpha, mach, V, CA] = readSheet(xlsxFile, "C_A");
    [alpha2, mach2, ~, CN] = readSheet(xlsxFile, "C_N");
    [alpha3, mach3, ~, CMP] = readSheet(xlsxFile, "C_MP");
    [~, ~, ~, FX] = readSheet(xlsxFile, "F_X_N");
    [~, ~, ~, MY] = readSheet(xlsxFile, "M_Y_nooff");

    fprintf("Checks (%d Mach x %d alpha):\n", numel(mach), numel(alpha));
    check(isequal(alpha, alpha2, alpha3) && isequal(mach, mach2, mach3), "Same Mach/alpha grid on every sheet");
    check(all(diff(mach) > 0) && all(diff(alpha) > 0), "Grids strictly increasing");
    check(all(isfinite([CA(:); CN(:); CMP(:)])), "No NaN/Inf in CA, CN, CMP");
    check(all(CN(:, alpha == 0) == 0) && all(CMP(:, alpha == 0) == 0), "CN = CMP = 0 at alpha = 0");

    % Back out Sref and Lref from the dimensional sheets (V >= 5 m/s is real CFD)
    q = 0.5 * rho * V(:).^2;
    use = V(:) >= 5;
    Sall = FX(use, :) ./ (q(use) .* CA(use, :));
    Sref = median(Sall(:), "omitnan");
    useM = use & any(abs(CMP) > 1e-3, 2);
    Lall = MY(useM, :) ./ (q(useM) * Sref .* CMP(useM, :));
    Lall = Lall(abs(CMP(useM, :)) > 1e-3);
    Lref = median(Lall, "omitnan");
    check(abs(Sref - pi/4 * 0.0793^2) / Sref < 1e-3, sprintf("Sref = %.6f m^2 (pi/4 d^2, d = 0.0793 m)", Sref));
    check(abs(Lref - 0.0793) / Lref < 1e-3, sprintf("Lref = %.4f m", Lref));

    S = struct("machGrid", mach(:).', "alphaGrid_deg", alpha(:).', "V_mps", V(:).', ...
        "CA", CA, "CN", CN, "CMP", CMP, "refArea_m2", Sref, "refLength_m", Lref, ...
        "notes", "From " + xlsxFile + ". Rows = Mach, columns = alpha (deg). CFD axes: +X nose->aft. " + ...
        "CA along +X, CN along +Z (with the crossflow air), CMP about +Y at the back of the gimbal, " + ...
        "positive = nose away from 0 AoA. No zero-alpha offset.");
    save(outFile, "-struct", "S", "-v7");
    R = load(outFile);
    check(all(cellfun(@(f) isequaln(R.(f), S.(f)), fieldnames(S))), "Round trip: reloaded " + outFile + " matches");
end

function [alpha, mach, V, val] = readSheet(file, sheet)
    % Header row starts with "Mach", then "V [m/s]", then the alphas
    C = readcell(file, "Sheet", sheet);
    isMach = cellfun(@(c) (ischar(c) || isstring(c)) && strcmp(string(c), "Mach"), C(:, 1));
    iHdr = find(isMach, 1);
    assert(~isempty(iHdr), "%s: no header row starting with 'Mach'", sheet);

    alpha = cellfun(@double, C(iHdr, 3:end));
    body = C(iHdr+1:end, :);
    body = body(cellfun(@(c) isnumeric(c) && isscalar(c), body(:, 1)), :);
    num = cellfun(@double, body);

    mach = num(:, 1);
    V = num(:, 2);
    val = num(:, 3:end);
end

function check(cond, msg)
    if cond
        fprintf("  [PASS] %s\n", msg);
    else
        fprintf("  [FAIL] %s\n", msg);
    end
end