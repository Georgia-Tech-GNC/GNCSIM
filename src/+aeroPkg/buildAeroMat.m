function S = buildAeroMat(inDir, outFile, refLength_in)
%BUILDAEROMAT 
% Parse RASAero II into  .mat of
% Has Mach x alpha coefficient grids
% Uses griddedInterpolant

%   Cm_tip = CN*Xcp/d 
%  

if nargin < 2 || isempty(outFile),      outFile = 'rasaero_aero.mat'; end
if nargin < 3 || isempty(refLength_in), refLength_in = 3.10;           end
d = refLength_in;

files = dir(fullfile(inDir, '*-alpha.txt'));
assert(~isempty(files), 'No *-alpha.txt files found in %s', inDir);

% Parse
P = struct('alpha', {}, 'data', {});
for k = 1:numel(files)
    fname = files(k).name;
    tok = regexp(fname, '(\d+(\.\d+)?)-alpha\.txt$', 'tokens', 'once');
    assert(~isempty(tok), 'Cannot read alpha from file name %s', fname);
    alphaFile = str2double(tok{1});

    txt   = fileread(fullfile(inDir, fname));
    lines = regexp(txt, '\r?\n', 'split');
    keep  = ~cellfun(@(s) isempty(strtrim(s)) || ~isempty(strfind(s, '-->')), lines);
    lines = lines(keep);                       
    assert(mod(numel(lines), 6) == 0, ...
        '%s: %d data lines, not a multiple of 6 (block layout changed?)', fname, numel(lines));

    nb = numel(lines) / 6;
    D  = nan(nb, 17);  
    for b = 1:nb
        L = cell(1, 6);
        for j = 1:6
            L{j} = sscanf(lines{(b-1)*6 + j}, '%f')';
        end
        M = L{1}(1);
        assert(all(cellfun(@(v) v(1), L) == M), '%s block %d: Mach differs between lines', fname, b);
        assert(numel(L{2}) == 3 && all(cellfun(@numel, L(3:4)) == 4) && all(cellfun(@numel, L(5:6)) == 6), ...
            '%s Mach %.2f: unexpected column count in lines 2-6', fname, M);
        alpha = L{3}(2);
        assert(abs(alpha - alphaFile) < 1e-9, '%s Mach %.2f: alpha %.2f does not match file name', fname, M, alpha);
        n1 = numel(L{1});
        if n1 >= 12, protub = L{1}(n1-1); else, protub = NaN; end   
        D(b, :) = [M, alpha, L{1}(3), L{1}(4), protub, L{1}(n1), ...   % Mach alpha CD0_off CD0_on CD_protub Re
                   L{2}(2), L{2}(3), ...                                % CNa_0to4 Xcp_0to4
                   L{3}(3), L{3}(4), ...                                % CN_pot CN_visc
                   L{4}(3), L{4}(4), ...                                % CN Xcp
                   L{5}(3:6), L{6}(6)];                                 % CL_off CD_off CN_off CA_off CA_on
        CNon(b) = L{6}(5); 
    end
    
    assert(max(abs(CNon(:) - D(:, 15))) < 1e-9, '%s: power-on CN differs from power-off CN', fname);
    clear CNon
    P(end+1) = struct('alpha', alphaFile, 'data', D); 
end
names = {'Mach','alpha','CD0_off','CD0_on','CD_protub','Re','CNa_0to4','Xcp_0to4', ...
         'CN_pot','CN_visc','CN','Xcp','CL_off','CD_off','CN_off','CA_off','CA_on'};
col = @(nm) find(strcmp(names, nm));

% Grids
[alphaGrid, order] = sort([P.alpha]);
P = P(order);
machGrid = P(1).data(:, 1)';
for k = 2:numel(P)
    assert(isequal(P(k).data(:, 1)', machGrid), 'Mach list in alpha=%g file differs from alpha=%g file', ...
        P(k).alpha, P(1).alpha);
end
assert(all(diff(machGrid) > 0) && all(diff(alphaGrid) > 0), 'Grids must be strictly increasing');

nM = numel(machGrid); nA = numel(alphaGrid);
G = struct();
for f = {'CN','Xcp','CA_on','CA_off','CN_pot','CN_visc','CL_off','CD_off','CD_protub'}
    G.(f{1}) = nan(nM, nA);
    for k = 1:nA, G.(f{1})(:, k) = P(k).data(:, col(f{1})); end
end
G.Cm_tip = G.CN .* G.Xcp / d;

% Vibe coded consistency check
tol = 2e-3;   
a   = repmat(deg2rad(alphaGrid), nM, 1);
chk = @(cond, msg) fprintf('  [%s] %s\n', ternary(cond, 'PASS', 'FAIL'), msg);
fprintf('Checks (%d Mach x %d alpha = %d points):\n', nM, nA, nM*nA);
c1 = max(abs(G.CN(:) - G.CN_pot(:) - G.CN_visc(:)));
chk(c1 < tol, sprintf('CN = CN_pot + CN_visc               (max err %.4f)', c1));
c2 = max(abs(G.CN(:) - P2(P, col('CN_off'))));
chk(c2 < 1e-9, sprintf('CN (line 4) = CN power-off (line 5) (max err %.4f)', c2));
c3 = max(abs(G.CA_off(:) - (G.CD_off(:).*cos(a(:)) - G.CL_off(:).*sin(a(:)))));
chk(c3 < tol, sprintf('CA = CD cos(a) - CL sin(a)          (max err %.4f)', c3));
c4 = max(abs(G.CN(:) - (G.CD_off(:).*sin(a(:)) + G.CL_off(:).*cos(a(:)))));
chk(c4 < tol, sprintf('CN = CD sin(a) + CL cos(a)          (max err %.4f)', c4));
i0 = find(alphaGrid == 0, 1);
if ~isempty(i0)
    chk(all(G.CN(:, i0) == 0), 'CN = 0 at alpha = 0');
end
sub = machGrid <= 0.8;
chk(all(all(diff(G.CN(sub, :), 1, 2) > 0)), 'CN increases with alpha (Mach <= 0.8)');
chk(all(isfinite(G.CN(:))) && all(isfinite(G.Xcp(:))) && all(isfinite(G.CA_on(:))), 'No NaN/Inf in CN, Xcp, CA');


fprintf('Spot check at Mach %.2f (compare with line 4 of that block in each file):\n', machGrid(3));
fprintf('  alpha   CN     Xcp(in)  Cm_tip   CA_off  CD_protub\n');
for k = 1:nA
    fprintf('  %5.1f  %6.3f  %6.3f  %7.3f  %6.3f  %6.3f\n', alphaGrid(k), G.CN(3,k), G.Xcp(3,k), ...
        G.Cm_tip(3,k), G.CA_off(3,k), G.CD_protub(3,k));
end
if any(G.CD_protub(sub, 1) > 0.2)
    warning('Protuberance CD is %.3f of total %.3f at Mach %.2f: check rail guide / protuberance inputs.', ...
        G.CD_protub(3,1), G.CA_off(3,1), machGrid(3));
end

% Save
S = struct('machGrid', machGrid, 'alphaGrid_deg', alphaGrid, ...
    'CN', G.CN, 'Xcp_in', G.Xcp, 'Cm_tip', G.Cm_tip, 'CA_on', G.CA_on, 'CA_off', G.CA_off, ...
    'CN_pot', G.CN_pot, 'CN_visc', G.CN_visc, 'CD_protub', G.CD_protub, ...
    'refLength_in', d, 'refArea_in2', pi*d^2/4, ...
    'notes', sprintf(['RASAero II Run Test files from %s. Rows = Mach, columns = alpha (deg). ' ...
        'Cm_tip = CN*Xcp/d about nose tip, positive when CP aft of tip (Niskanen 2013 Eq. 3.4).'], inDir));
save(outFile, '-struct', 'S', '-v7');
R = load(outFile);
same = all(cellfun(@(f) isequaln(R.(f), S.(f)), fieldnames(S)));
chk(same, sprintf('Round trip: reloaded %s matches', outFile));
end

function v = P2(P, c)
% stack column c of every alpha file into one vector in grid order
v = cell2mat(arrayfun(@(p) p.data(:, c), P, 'UniformOutput', false));
v = v(:);
end

function out = ternary(cond, a, b)
if cond, out = a; else, out = b; end
end