function result = checkTrajOptDynamics(rocketDynamics, environment, opts)
%CHECKTRAJOPTDYNAMICS  Sanity-check the traj-opt dynamics wrapper with no optimizer.
%
%   result = checkTrajOptDynamics(rocketDynamics, environment)
%       Checks output size / finiteness, integrates with constant control from
%       ignition to burnout, and tests that the wrapper is vectorized.
%
%   Options (name-value):
%       tiltDeg     Initial pitch tilt off vertical [deg]. Default 0.
%                   Ignored when a log is supplied.
%       control     Constant 3x1 control input [roll; pitch; yaw]. Default zeros.
%       tLog, xLog  1xM time [s] and 14xM states from an OPEN-LOOP flight sim
%                   run (flight.runClosedLoop = false). The integration then
%                   starts from the logged state at rail exit and is compared
%                   against the log.
%       railLength  Launch rail length [m], used to find rail exit. Default 5.
%       tEnd        End time [s]. Default is burnout. If set later than
%                   burnout, the integration stops by itself at apogee.
%
%   Examples:
%       checkTrajOptDynamics(rocketDynamics, environment, tiltDeg=2)
%       checkTrajOptDynamics(rocketDynamics, environment, control=[0; 0; 1])
%       checkTrajOptDynamics(rocketDynamics, environment, tLog=t, xLog=x)

    arguments
        rocketDynamics
        environment
        opts.tiltDeg (1,1) double = 0
        opts.control (3,1) double = [0; 0; 0]
        opts.tLog double = []
        opts.xLog double = []
        opts.railLength (1,1) double = 5.0
        opts.tEnd (1,1) double = NaN
    end

    nState = 14;
    nControl = 3;
    stateNames = {'mass','x','y','z','vx','vy','vz','q0','q1','q2','q3','wx','wy','wz'};
    uConst = opts.control;
    tLog = opts.tLog;
    xLog = opts.xLog;

    tIgn = rocketDynamics.tIgnition;
    tBurnout = tIgn + rocketDynamics.burnTime;
    tEnd = tBurnout;
    if ~isnan(opts.tEnd), tEnd = opts.tEnd; end

    % Same frozen ground-level atmosphere and zero wind as setupNominalTrajectory
    [T, a, P, rho] = environment.getAtmosphere(0);
    envConditions = flightPkg.AtmosphericConditions(rho, a, P, T, [0, 0, 0]);

    % Same call OptimTraj will make: dyn(t, x, u)
    dyn = @(t, x, u) rocketDynamics.rocketModelTrajOptWrapper(x, u, envConditions, t, ...
        flightPkg.FlightState.POWERED_ASCENT, true, true);

    %% Initial state
    haveLog = ~isempty(tLog);
    if haveLog
        assert(size(xLog, 1) == nState, 'xLog must be 14xM.');
        [tLog, iu] = unique(tLog(:).');
        xLog = xLog(:, iu);
        relAlt = xLog(4, :) - xLog(4, 1);   % log altitude may include ground elevation
        i0 = find(tLog >= tIgn & relAlt > opts.railLength, 1);
        assert(~isempty(i0), 'Log never clears the rail after ignition.');
        t0 = tLog(i0);
        x0 = xLog(:, i0);
        tEnd = min(tEnd, tLog(end));
    else
        quat0 = Utils.eulerToQuat([0, -90 + opts.tiltDeg, 0]).';
        x0 = [rocketDynamics.massWet, [0, 0, 0], [1, 0, 0], quat0, [0, 0, 0]].';
        t0 = tIgn;
    end

    fprintf('\nRun: tilt = %.2f deg, control = [%g; %g; %g], log = %d\n', ...
        opts.tiltDeg * ~haveLog, uConst(1), uConst(2), uConst(3), haveLog);

    %% Check 1: single-point call
    dx0 = dyn(t0, x0, uConst);
    assert(isequal(size(dx0), [nState, 1]), ...
        'Wrapper returned %dx%d, expected %dx1.', size(dx0, 1), size(dx0, 2), nState);
    assert(all(isfinite(dx0)), 'Non-finite derivative at t0: %s', ...
        strjoin(stateNames(~isfinite(dx0)), ', '));

    fprintf('\n--- Check 1: derivative at t = %.2f s ---\n', t0);
    for k = 1:nState
        fprintf('  d(%-4s)/dt = % .5g\n', stateNames{k}, dx0(k));
    end
    if ~haveLog
        fprintf('  Note: thrust is zero exactly at ignition, so d(vx)/dt = -g and zero mass flow are normal here.\n');
    end

    %% Check 2: constant-control integration
    odeFun = @(t, x) reshape(dyn(t, x, uConst), [], 1);
    odeOpts = odeset('RelTol', 1e-8, 'AbsTol', 1e-8, 'MaxStep', 0.02, ...
        'Events', @(t, x) apogeeEvent(t, x, odeFun, tBurnout));
    [tSim, xSim] = ode45(odeFun, [t0, tEnd], x0, odeOpts);
    tSim = tSim.';
    xSim = xSim.';

    assert(all(isfinite(xSim), 'all'), 'Integration produced NaN/Inf (first at t = %.3f s).', ...
        tSim(find(any(~isfinite(xSim), 1), 1)));

    speed = vecnorm(xSim(5:7, :));
    quatNorm = vecnorm(xSim(8:11, :));
    omegaPeak = max(abs(xSim(12:14, :)), [], 2);

    % Rotation angle away from the initial attitude (insensitive to q vs -q)
    qDot0 = abs(sum(xSim(8:11, :) .* x0(8:11), 1)) ./ (quatNorm * norm(x0(8:11)));
    attDevDeg = 2 * acosd(min(qDot0, 1));

    fprintf('\n--- Check 2: integration, t = %.2f to %.2f s ---\n', tSim(1), tSim(end));
    fprintf('  Altitude gain:        %.1f m\n', xSim(4, end) - xSim(4, 1));
    fprintf('  Final speed:          %.1f m/s (max %.1f)\n', speed(end), max(speed));
    fprintf('  Final mass:           %.3f kg (massDry = %.3f)\n', xSim(1, end), rocketDynamics.massDry);
    if tSim(end) > tBurnout
        iBurn = find(tSim >= tBurnout, 1);
        fprintf('  Coast phase:          mass change %.4f kg (should be ~0), speed at burnout %.1f m/s\n', ...
            xSim(1, end) - xSim(1, iBurn), speed(iBurn));
        if tSim(end) < tEnd - 1e-6
            fprintf('  Apogee:               t = %.2f s, %.1f m above start, speed there %.1f m/s\n', ...
                tSim(end), xSim(4, end) - xSim(4, 1), speed(end));
        else
            fprintf('  Apogee not reached by tEnd = %.1f s.\n', tEnd);
        end
    end
    fprintf('  Quaternion norm:      %.6f to %.6f\n', min(quatNorm), max(quatNorm));
    fprintf('  Attitude change:      max %.3f deg, final %.3f deg from initial\n', max(attDevDeg), attDevDeg(end));
    fprintf('  Peak |wx|,|wy|,|wz|:  %.4g, %.4g, %.4g rad/s\n', omegaPeak(1), omegaPeak(2), omegaPeak(3));
    fprintf('  Final wx, wy, wz:     %.4g, %.4g, %.4g rad/s\n', xSim(12, end), xSim(13, end), xSim(14, end));

    %% Check 3: vectorized call (OptimTraj passes t as 1xN, x as 14xN, u as 3xN)
    idx = unique(round(linspace(1, numel(tSim), 25)));
    tG = tSim(idx);
    xG = xSim(:, idx);
    uG = repmat(uConst, 1, numel(idx));

    dxLoop = zeros(nState, numel(idx));
    for k = 1:numel(idx)
        dxLoop(:, k) = dyn(tG(k), xG(:, k), uG(:, k));
    end

    fprintf('\n--- Check 3: vectorized call ---\n');
    vectorized = false;
    try
        dxVec = dyn(tG, xG, uG);
        if ~isequal(size(dxVec), size(dxLoop))
            fprintf('  FAIL: returned %dx%d, expected %dx%d.\n', ...
                size(dxVec, 1), size(dxVec, 2), nState, numel(idx));
        else
            vecErr = max(abs(dxVec - dxLoop), [], 'all');
            vectorized = vecErr <= 1e-8 * max(1, max(abs(dxLoop), [], 'all'));
            if vectorized
                fprintf('  PASS: matches column-by-column calls (max diff %.2g).\n', vecErr);
            else
                [~, worst] = max(max(abs(dxVec - dxLoop), [], 2));
                fprintf('  FAIL: differs from column-by-column calls (max diff %.3g, worst state %s).\n', ...
                    vecErr, stateNames{worst});
            end
        end
    catch ME
        fprintf('  FAIL: wrapper errored on matrix input: %s\n', ME.message);
    end

    %% Check 4: compare against the flight-sim log
    posErr = []; velErr = []; attErrDeg = []; xRef = [];
    if haveLog
        xRef = interp1(tLog.', xLog.', tSim.').';
        posErr = vecnorm(xSim(2:4, :) - xRef(2:4, :));
        velErr = vecnorm(xSim(5:7, :) - xRef(5:7, :));
        qDot = abs(sum(xSim(8:11, :) .* xRef(8:11, :), 1)) ./ (quatNorm .* vecnorm(xRef(8:11, :)));
        attErrDeg = 2 * acosd(min(qDot, 1));

        fprintf('\n--- Check 4: wrapper vs flight sim (from rail exit) ---\n');
        fprintf('  Position error:  max %.2f m, final %.2f m\n', max(posErr), posErr(end));
        fprintf('  Velocity error:  max %.2f m/s, final %.2f m/s\n', max(velErr), velErr(end));
        fprintf('  Attitude error:  max %.3f deg, final %.3f deg\n', max(attErrDeg), attErrDeg(end));
        fprintf('  Mass error:      max %.4f kg\n', max(abs(xSim(1, :) - xRef(1, :))));
    end

    %% Plots
    figure('Name', sprintf('Traj-opt dynamics check (tilt %.1f deg, u = [%g %g %g])', ...
        opts.tiltDeg * ~haveLog, uConst(1), uConst(2), uConst(3)));

    subplot(4, 1, 1); hold on; grid on;
    plot(tSim, xSim(4, :), 'LineWidth', 1.5);
    if haveLog
        plot(tSim, xRef(4, :), '--', 'LineWidth', 1.5);
        legend('wrapper + ode45', 'flight sim', 'Location', 'best');
    end
    ylabel('z [m]');

    subplot(4, 1, 2); hold on; grid on;
    plot(tSim, speed, 'LineWidth', 1.5);
    if haveLog, plot(tSim, vecnorm(xRef(5:7, :)), '--', 'LineWidth', 1.5); end
    ylabel('speed [m/s]');

    subplot(4, 1, 3); hold on; grid on;
    plot(tSim, attDevDeg, 'LineWidth', 1.5);
    if haveLog
        plot(tSim, attErrDeg, '--', 'LineWidth', 1.5);
        legend('rotation from initial', 'error vs flight sim', 'Location', 'best');
    end
    ylabel('attitude [deg]');

    subplot(4, 1, 4); hold on; grid on;
    plot(tSim, rad2deg(xSim(12:14, :)), 'LineWidth', 1.5);
    legend('wx', 'wy', 'wz', 'Location', 'best');
    ylabel('body rates [deg/s]');
    xlabel('time [s]');

    %% Output
    result.time = tSim;
    result.states = xSim;
    result.control = repmat(uConst, 1, numel(tSim));
    result.vectorized = vectorized;
    result.attDevDeg = attDevDeg;
    result.posErr = posErr;
    result.velErr = velErr;
    result.attErrDeg = attErrDeg;
    result.tBurnout = tBurnout;
end

function [value, isterminal, direction] = apogeeEvent(t, x, odeFun, tBurnout)
    % Stop at apogee (vertical velocity crosses zero). Armed only after burnout.
    value = 1;
    if t > tBurnout
        dx = odeFun(t, x);
        value = dx(4);
    end
    isterminal = 1;
    direction = -1;
end