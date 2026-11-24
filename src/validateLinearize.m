%% validateLinearize
% Run AFTER setupMuSynthesis (needs `controller` and `rocketDynamics` in the workspace).
% Part 1: which design plant matches the nonlinear model? (independent of the controller design)
% Part 2: do the designed gains actually stabilize the full 3-state ROM, and with what margins?
%         Run Part 2 once per linearizeMode and compare the printed tables.

bpIdx   = [2 4 6 8];     % breakpoints to check
stepDeg = 0.5;           % gimbal step for the open-loop test [deg]
Tend    = 1.0;           % open-loop horizon [s]

env = controller.linearizer.envConditions;
fs  = flightPkg.FlightState.POWERED_ASCENT;
bp  = controller.breakPoints;

% Same straight-up operating point the controller linearizes about
vert = @(x) [x(1:4); norm(x(5:7)); 0; 0; reshape(Utils.eulerToQuat([0 -90 0]), [], 1); 0; 0; 0];

%% Part 1: open-loop step response, nonlinear vs 3-state vs 2-state
figure('Name', 'Design plant vs nonlinear');
for k = 1:numel(bpIdx)
    i = bpIdx(k);  t = bp.time(i);
    xv = vert(bp.state{i});

    [A, B] = controller.linearizer.calculateROMpitch(xv, 0, bp.actuatorState{i}, t, false);
    Bd = B(:, 1) * (180/pi);                          % per-degree -> per-radian
    G3 = minreal(ss(A, Bd, [0 1 0], 0), 1e-6, false); % what the old linearize built
    G2 = ss(A(1:2,1:2), Bd(1:2), [0 1], 0);           % [alpha; q] only

    % Nonlinear: gimbal step minus the unperturbed run (the operating point is not a trim point)
    mk = @(u) @(~, xx) rocketDynamics.rocketModelControlWrapper(xx, u, env, t, fs, true, true);
    [t0, X0] = ode45(mk([0; 0; 0]),       [0 Tend], xv);
    [t1, X1] = ode45(mk([0; stepDeg; 0]), [0 Tend], xv);
    tg  = linspace(0, Tend, 200).';
    qNL = interp1(t1, X1(:, 13), tg) - interp1(t0, X0(:, 13), tg);   % omega_y

    qL3 = step(G3, tg) * deg2rad(stepDeg);
    qL2 = step(G2, tg) * deg2rad(stepDeg);

    fprintf('bp %d (t=%.2fs): Gp0 order=%d | rel. error vs nonlinear: 3-state %.1f%%, 2-state %.1f%% | open-loop poles(3-state): %s\n', ...
        i, t, order(G3), 100*norm(qL3-qNL)/norm(qNL), 100*norm(qL2-qNL)/norm(qNL), mat2str(pole(G3).', 3));

    subplot(2, 2, k);
    plot(tg, rad2deg(qNL), 'k', tg, rad2deg(qL3), 'b--', tg, rad2deg(qL2), 'r:', 'LineWidth', 1.5);
    title(sprintf('bp %d, t=%.1fs', i, t));  xlabel('s');  ylabel('\Delta q [deg/s]');  grid on;
    if k == 1, legend('nonlinear', '3-state', '2-state', 'Location', 'best'); end
end

%% Part 2: designed gains on the full 3-state ROM (+ servo + delay), no duplicated integrator
r = controller.results;
[nT, dT] = pade(controller.timeLag, 2);
Dl = tf(nT, dT);
Gs = tf(1, [controller.servoTau 1]);
fprintf('\n%-4s %-8s %-6s %-6s %-14s %-10s %-10s\n', 'bp', 't [s]', 'wn', 'zeta', 'max Re(pole)', 'GM [dB]', 'PM [deg]');
for i = find(~isnan(r.wn))
    xv = vert(bp.state{i});
    [A, B] = controller.linearizer.calculateROMpitch(xv, 0, bp.actuatorState{i}, bp.time(i), false);
    Bd = B(:, 1) * (180/pi);
    P  = ss(A, Bd, [0 1 0; 0 0 1], 0) * (Gs * Dl);    % command -> [q; theta]
    K  = [r.Kd(i), r.Kp(i)];                          % u = -Kd*q - Kp*theta
    CL = feedback(P, K);
    [Gm, Pm] = margin(K * P);
    z = NaN;  if isfield(r, 'zeta'), z = r.zeta(i); end
    fprintf('%-4d %-8.2f %-6.1f %-6.2f %-14.3g %-10.1f %-10.1f\n', ...
        i, bp.time(i), r.wn(i), z, max(real(pole(CL))), 20*log10(Gm), Pm);
end