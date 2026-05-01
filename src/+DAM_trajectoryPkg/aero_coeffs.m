function [CN, CA, CMR, CMP] = aero_coeffs(M, alpha, beta)
% Alpha (aoa) and beta (fin tab deflection) in radians
% M is mach number

% Prandtl-Glauert Correction
PG = (1 - M.^2).^(-0.5);

% NORMAL FORCE COEFFICIENT (CN)
CN = PG.*((2.9367 .* alpha) + (1.2593 .* beta) + (0.7397 .* alpha .* beta));

% AXIAL FORCE COEFFICIENT (CA)
CA = (0.009039 .* M.^(-0.5431)) + (0.029015 .* alpha) + (0.176115 .* beta) + 0.236954;

% ROLLING MOMENT COEFFICIENT (CMR)
CMR = (PG .* (-0.8104 .* alpha + 0.4382 .* beta)) + (-2.2707 .* alpha.^2) + (-0.3259 .* beta.^2) + (-1.1520 .* alpha) + (-1.4777 .* beta) + (-1.4807 .* alpha .* beta) + (14.0089 .* alpha.^2 .* beta) + (-6.8365 .* alpha .* beta.^2);

% PITCHING MOMENT COEFFICIENT (CMP)
CMP = PG .* (-6.3974 .* alpha + 0.5260 .* beta) + (-3.0457 .* alpha.^2) + (-8.5399 .* beta.^2) + (-9.8052 .* alpha) + (-6.8605 .* beta) + (-2.9308 .* alpha .* beta) + (-4.1758 .* alpha.^2 .* beta) + (7.2513 .* alpha .* beta.^2);
end