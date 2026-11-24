% Gain-scheduled PD controller for the Lorax (HPR-TVC), tuned against a robust-performance spec.
%
% Simplification of the old mu-synthesis version. musyn gave mu(full) ~ 0.52 but produced a ~21st-order
% controller that a PD could not imitate (fit error 65-80%, mu(PD) ~ 4.4). So instead of
% "synthesize a big controller, then curve-fit a PD", this tunes the PD directly:
%
%   For each breakpoint:
%     1. Linearize pitch -> Gp0 (command [rad] -> pitch rate), plus Mdelta (control authority) and
%        Malpha (aero instability).
%     2. Build the uncertain closed loop with the REAL control structure: u = Kp*(r - theta) - Kd*rate
%        (the reference only goes through Kp; the rate comes from the gyro, so there is no differentiator).
%     3. For each candidate bandwidth wn, set Kp, Kd in closed form
%            Kp = (Malpha + wn^2)/Mdelta,   Kd = 2*zeta*wn/Mdelta
%        and score it by the WORST weighted H-infinity norm over the nominal plant plus random samples
%        of the uncertainty (< 1 means the spec is met for every sampled plant).
%     4. Keep the wn with the lowest worst-case score.
%
% The Lorax is axisymmetric, so the pitch gains are also applied to yaw. Roll has no actuator.
% Breakpoints with almost no TVC authority (before ignition / after burnout) copy the nearest designed one.
%
% Runtime law (same as TVCPID):
%   u [rad] = Kp*err - Kd*(omega - omegaRef)      err = body-frame pointing error
%   rocketControl [deg] = rad2deg(u)
%
% breakPoints.gain{i} = { [Kp 0 Kd] }   (interpolated by BreakPoints.interpolateGain)
%
% Requires: Control System Toolbox + Robust Control Toolbox (for ultidyn / usample).

classdef MuSynthesisController < controllerPkg.Controller

    properties
        breakPoints
        referenceAttitudes % (1+4) x n : row 1 = time,rows 2:5 = reference quaternion [w;x;y;z]
        useActuatorModel = true
        linearizer % controllerPkg.ReduceLTI

        % State
        quatIdx (1,4) double = 8:11
        omegaIdx (1,3) double = 12:14
        velIdx (1,3) double = 5:7

        minControlAuthority = 0.05 %rad/s^2 per deg. Breakpoints below this are not built

        % Design settings
        zeta = 0.7 % damping
        wnGrid = linspace(1, 12, 23) % candidate bandwidths [rad/s] (servo is ~20 rad/s, keep wn well below)
        nSamples = 20 % uncertainty samples per candidate
        servoTau = 0.05 % [s] overwritten from the actuator model
        timeLag = 0.01 % [s] pure delay in the design plant (Pade order 2)
        weights struct % see defaultWeights()

        results struct = struct() % per-breakpoint design summary (filled by populateBreakPoints)

        % Runtime state
        integralError (3,1) double = zeros(3,1)
        lastTime (1,1) double = 0
    end

    methods
        function obj = MuSynthesisController(breakPoints, referenceAttitudes, linearizer, varargin)
            obj.breakPoints = breakPoints;
            obj.referenceAttitudes = referenceAttitudes;
            obj.linearizer = linearizer;
            obj.weights = obj.defaultWeights();
            try
                am = linearizer.rocketDynamics.actuatorModel;
                obj.servoTau = am.timeConstant;
                obj.weights.uMax = deg2rad(am.limits.maxDeflection);
                obj.weights.uCorner = am.limits.maxRate / am.limits.maxDeflection;
            catch ME
                warning('MuSynthesisController:actuatorSync','Could not read actuatorModel settings (%s). Using placeholders.', ME.message);
            end
            for k = 1:2:numel(varargin)
                obj.(varargin{k}) = varargin{k+1};
            end
        end

        function rocketControl = getControl(obj, state, currentCumVelocity, currentTime)
            arguments
                obj
                state (:,1) double
                currentCumVelocity (1,1) double {mustBeNonnegative}
                currentTime (1,1) double {mustBeNonnegative}
            end
            [gains, controlRef] = obj.breakPoints.interpolateGain(currentCumVelocity);
            g = gains{1}; % [Kp Ki Kd]
            ref = deg2rad(controlRef);

            [qRef, omegaRef] = obj.getReference(currentTime);
            err = obj.getError(state, currentTime, qRef);
            omega = state(obj.omegaIdx) - omegaRef;

            u = zeros(3, 1); % [roll; pitch; yaw] in rad
            u(2) = ref(2) + g(1)*err(2) - g(3)*omega(2); % pitch
            u(3) = ref(3) - (g(1)*err(3) - g(3)*omega(3)); % yaw uses the pitch gains
            rocketControl = rad2deg(u);
            obj.lastTime = currentTime;
        end

        function err = getError(obj, rocketStateEstimate, currentTime, qRef)
            % Body frame pointing error using small angle aprrox
            arguments
                obj
                rocketStateEstimate (:,1) double
                currentTime (1,1) double {mustBeNonnegative}
                qRef double = []
            end
            q_cur = Utils.normalizeQuat(rocketStateEstimate(obj.quatIdx));
            if isempty(qRef)
                qRef = obj.getReference(currentTime);
            end
            q_ref = Utils.normalizeQuat(qRef(:));
            q_cur_inv = [q_cur(1); -q_cur(2); -q_cur(3); -q_cur(4)];

            vb = [1; 0; 0];
            v_curr = obj.quatRotate(q_cur(2:4), q_cur(1), vb);
            v_desired = obj.quatRotate(q_ref(2:4), q_ref(1), vb);

            d = dot(v_curr, v_desired);
            c = cross(v_curr, v_desired);
            q_err = [1 + d; c(1); c(2); c(3)];
            nrm = norm(q_err);
            if nrm < 1e-12
                q_err = [0; 0; 1; 0];
            else
                q_err = q_err / nrm;
            end
            error_body = obj.quatRotate(q_cur_inv(2:4), q_cur_inv(1), q_err(2:4));
            err = 2 * error_body(:);
        end

        function [qRef, omegaRef] = getReference(obj, currentTime)
            timeVec = obj.referenceAttitudes(1, :);
            [~, k] = min(abs(timeVec - currentTime));
            qRef = obj.referenceAttitudes(2:5, k);
            omegaRef = zeros(3, 1);
        end

        function resetState(obj)
            obj.integralError = zeros(3, 1);
            obj.lastTime = 0;
        end

        function populateBreakPoints(obj)
            % Only call this function before running flight
            n = obj.breakPoints.numBreakPoints;
            Kp = nan(1, n);  Kd = nan(1, n);  wnBest = nan(1, n);  gam = nan(1, n);  auth = zeros(1, n);

            for i = 1:n
                time = obj.breakPoints.time(i);
                x = obj.verticalState(obj.breakPoints.state{i}); % linearize about straight up flight
                [Gp0, A, B] = obj.linearize(x, obj.breakPoints.controlInput{i}, obj.breakPoints.actuatorState{i}, time);
                obj.breakPoints.stateMatrix{i} = {A};
                obj.breakPoints.controlMatrix{i} = {B};

                auth(i) = abs(B(2, 1));
                if auth(i) < obj.minControlAuthority
                    fprintf('bp %d/%d (t=%.2fs): skipped, authority %.3g < %.3g\n', i, n, time, auth(i), obj.minControlAuthority);
                    continue;
                end
                [Kp(i), Kd(i), wnBest(i), gam(i)] = obj.designPD(Gp0, A, B);
                fprintf('bp %d/%d (t=%.2fs): wn=%.1f rad/s  Kp=%.4g  Kd=%.4g  worst-case gamma=%.3f\n',i, n, time, wnBest(i), Kp(i), Kd(i), gam(i));
            end

            % Skipped breakpoints copy the nearest designed one
            designed = find(~isnan(Kp));
            if isempty(designed)
                error('MuSynthesisController:noDesign', 'No breakpoint has enough control authority to design.');
            end
            for i = 1:n
                if isnan(Kp(i))
                    [~, k] = min(abs(designed - i));
                    j = designed(k);
                    Kp(i) = Kp(j);  Kd(i) = Kd(j);
                end
                obj.breakPoints.gain{i} = {[Kp(i), 0, Kd(i)]};
            end

            obj.results = struct('time', obj.breakPoints.time, 'wn', wnBest, 'Kp', Kp, 'Kd', Kd, ...
                                 'worstGamma', gam, 'authority', auth);
        end

        function report(obj)
            r = obj.results;
            for i = 1:numel(r.time)
                if isnan(r.worstGamma(i))
                    fprintf('bp %2d t=%6.2fs  copied (authority %.3g)       Kp=%.4g Kd=%.4g\n', i, r.time(i), r.authority(i), r.Kp(i), r.Kd(i));
                else
                    fprintf('bp %2d t=%6.2fs  wn=%4.1f  Kp=%.4g Kd=%.4g  worst-case gamma=%.3f %s\n', ...
                        i, r.time(i), r.wn(i), r.Kp(i), r.Kd(i), r.worstGamma(i), obj.verdict(r.worstGamma(i)));
                end
            end
        end

        function [CL, blocks] = buildClosedLoop(obj, Gp0, Kp, Kd)
            % Uncertain closed loop with u = Kp*(r - theta - noise) - Kd*rate.
            % Inputs : r, dist, noise   Outputs: zp, zu
            w = obj.weights;
            Wr = tf(w.refMag);
            Wr.u = 'r';
            Wr.y = 'rs';
            Wd = tf(w.distMag);
            Wd.u = 'dist';
            Wd.y = 'wdist';
            Wn = tf(w.noiseMag);
            Wn.u = 'noise';
            Wn.y = 'wn';
            Wp = makeweight(w.trackDC, [w.trackBw 1], w.trackHF);
            Wp.u = 'ep';
            Wp.y = 'zp';
            Wu = makeweight(0.2/w.uMax, [w.uCorner 1/w.uMax], 3/w.uMax);
            Wu.u = 'dcmd';
            Wu.y = 'zu';

            % Servo (first-order lag) with multiplicative uncertainty
            Gs0 = tf(1, [obj.servoTau 1]);
            Wus = makeweight(w.servoUncDC, [3/obj.servoTau 1], w.servoUncHF);
            Ds  = ultidyn('Delta_s', [1 1]);
            Gs  = Gs0 * (1 + Wus*Ds); 
            Gs.u = 'dcmd';  
            Gs.y = 'dbar';

            % Pure delay (Pade)
            [nT, dT] = pade(obj.timeLag, 2);
            T = tf(nT, dT);               
            T.u = 'dbar';   
            T.y = 'dtime';

            % Airframe with multiplicative uncertainty
            Wup = makeweight(w.airUncDC, [w.airUncFreq 1], w.airUncHF);
            Dp = ultidyn('Delta_p', [1 1]);
            Gp = Gp0 * (1 + Wup*Dp);     
            Gp.u = 'dsum';  
            Gp.y = 'w';

            % Attitude = integral of rate
            Itrue = tf(1, [1 0]);         
            Itrue.u = 'w';  
            Itrue.y = 'theta';

            % Controller: attitude error through Kp, gyro rate through Kd
            Kpb = ss(Kp);  
            Kpb.u = 'e';  
            Kpb.y = 'up';
            Kdb = ss(Kd);  
            Kdb.u = 'w';  
            Kdb.y = 'ud';

            S1 = sumblk('e = rs - theta - wn');
            S2 = sumblk('dsum = dtime + wdist');
            S3 = sumblk('dcmd = up - ud');
            S4 = sumblk('ep = rs - theta');

            blocks = struct('Wr', Wr, 'Wp', Wp, 'Wu', Wu, 'Wd', Wd, 'Wn', Wn, 'Gs', Gs, 'T', T, 'Gp', Gp);
            CL = connect(Wr, Wd, Wn, Wp, Wu, Gs, T, Gp, Itrue, Kpb, Kdb, S1, S2, S3, S4, {'r','dist','noise'}, {'zp','zu'});
        end
    end

    methods (Access = private)
        function [Kp, Kd, wnBest, gammaBest] = designPD(obj, Gp0, A, B)
            Mdelta = B(2, 1) * (180/pi); % rad/s^2 per rad of gimbal
            Malpha = max(A(2, 1), 0); % aero instability (rad/s^2 per rad of alpha)

            % Draw the uncertainty samples once and reuse them for every candidate
            CL0 = obj.buildClosedLoop(Gp0, 1, 1);
            [~, samples] = usample(CL0, obj.nSamples);

            Kp = NaN; 
            Kd = NaN; 
            wnBest = NaN; 
            gammaBest = Inf;
            for wn = obj.wnGrid
                kp = (Malpha + wn^2) / Mdelta;
                kd = 2 * obj.zeta * wn / Mdelta;
                CL = obj.buildClosedLoop(Gp0, kp, kd);
                g = obj.worstGamma(CL, samples);
                if g < gammaBest
                    gammaBest = g;  
                    Kp = kp;  
                    Kd = kd;  
                    wnBest = wn;
                end
            end
            if ~isfinite(gammaBest)
                % Nothing in the grid stabilizes every sample: use the middle of the grid and say so.
                wnBest = obj.wnGrid(ceil(end/2));
                Kp = (Malpha + wnBest^2) / Mdelta;
                Kd = 2 * obj.zeta * wnBest / Mdelta;
                warning('MuSynthesisController:noStableCandidate', ...
                    'No candidate wn stabilized all uncertainty samples; using wn=%.1f. Check weights/uncertainty.', wnBest);
            end
        end

        function g = worstGamma(~, CL, samples)
            % Largest weighted H-infinity norm over the nominal loop and all samples
            % (Inf if any of them is unstable). Below 1 means the spec is met.
            nom = CL.NominalValue;
            if ~isstable(nom)
                g = Inf;
                return;
            end
            CLs = usubs(CL, samples);
            if ~all(reshape(isstable(CLs), [], 1))
                g = Inf;
                return;
            end
            g = max([norm(nom, inf); reshape(norm(CLs, inf), [], 1)]);
        end

        function [Gp0, A, B] = linearize(obj, state, controlInput, actuatorState, time)
            % Command -> pitch rate from ReduceLTI. Servo is NOT in this model (it is Gs in the design plant).
            [A, B] = obj.linearizer.calculateROMpitch(state, controlInput(2), actuatorState, time, false);
            Bd = B(:, 1) * (180/pi); % per-degree -> per-radian
            C = [0 1 0]; % pitch ROM states = [alpha; q; theta_err]
            Gp0 = minreal(ss(A, Bd, C, 0), 1e-6, false);
        end

        function xv = verticalState(obj, x)
            % Same mass/position/speed, but pointing straight up with no rotation
            xv = x;
            xv(obj.velIdx) = [norm(x(obj.velIdx)); 0; 0];
            q = Utils.eulerToQuat([0, -90, 0]);
            xv(obj.quatIdx) = q(:);
            xv(obj.omegaIdx) = 0;
        end

        function s = verdict(~, g)
            if g < 1
                s = '(spec met)';
            else
                s = '(spec NOT met)';
            end
        end
    end

    methods (Static)
        function v_rotated = quatRotate(qv, qw, v)
            t = 2.0 * cross(qv, v);
            v_rotated = v + (qw * t) + cross(qv, t);
        end

        function w = defaultWeights()
            % uMax and uCorner are overwritten from the actuator model in the constructor.
            w.refMag = deg2rad(5); % max attitude command magnitude
            w.distMag = deg2rad(0.5); % equivalent gimbal disturbance
            w.noiseMag = deg2rad(0.5); % attitude measurement noise
            w.uMax = deg2rad(10); % gimbal deflection limit
            w.uCorner = 10; % freq where control penalty rolls up [rad/s]
            w.trackDC = 50; % low-freq gain of Wp
            w.trackBw = 3; % desired tracking bandwidth [rad/s]
            w.trackHF = 0.5; % high-freq gain of Wp
            w.servoUncDC = 0.10; % servo uncertainty at DC
            w.servoUncHF = 2; % servo uncertainty at high frequency
            w.airUncDC = 0.30; % airframe uncertainty at DC
            w.airUncFreq = 2*pi*30; % airframe uncertainty reaches 100% here [rad/s]
            w.airUncHF = 2;
        end
    end
end