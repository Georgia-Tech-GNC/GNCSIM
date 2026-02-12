classdef SimpleNavigation
    % SimpleNavigation  Simple discrete-time navigation + roll controller
    %   No EKF. Adds IMU noise + bias, low-pass filters IMU rate, delays it,
    %   and produces a PID airframe torque command with clamping, anti-windup,
    %   and wheel-speed torque derating.
    %
    %   Public usage:
    %     nav = navigationPkg.SimpleNavigation(params);
    %     estState = nav.getStateEstimate(state, stateDot, time, log);
    %
    %   The 'params' struct may contain fields below; defaults are provided.

    properties
        % IMU / filtering
        sampleTime = 0.01;           % s (update period)
        imuNoiseStd = 0.005;         % rad/s measurement noise std
        imuBias = 0.0;               % rad/s constant bias
        lpfTau = 0.02;               % LPF time constant (s)
        delaySteps = 2;              % integer steps of delay buffer

        % PID gains
        Kp = 1.0;
        Ki = 0.5;
        Kd = 0.01;
        dOnMeasurement = true;       % true -> derivative on measurement

        % Integrator state and bounds
        I = 0.0;                     % integrator state
        Imin = -inf;
        Imax = +inf;

        % Anti-windup / torque limits
        torqueLimit = 1.0;           % maximum reaction wheel torque (Nm)

        % Wheel speed limits (rad/s)
        wheelSpeed = 0.0;            % current wheel speed (rad/s) - user/encoder updated
        omega_max = 1000;            % physical encoder/ESC limit (rad/s)
        omega_soft = [];             % soft-start threshold; if empty, set 0.6*omega_max

        % Internal filter & buffer state
        lpfState = 0.0;              % LPF state (filtered imu rate)
        delayBuffer = [];            % circular buffer for delayed measurements

        % Previous values for discrete derivative
        prevError = 0.0;
        prevMeas = 0.0;

        % Configuration flags
        seed = 0;                    % random seed
        enableNoise = true;
        enableBiasRandomWalk = false;
        biasDriftStd = 0.0;          % if random walk enabled (rad/s per sqrt(s))
    end

    methods
        function obj = SimpleNavigation(params)
            if nargin>0 && isstruct(params)
                f = fieldnames(params);
                for i=1:numel(f)
                    if isprop(obj, f{i}); obj.(f{i}) = params.(f{i}); end
                end
            end
            if isempty(obj.omega_soft)
                obj.omega_soft = 0.6 * obj.omega_max; % conservative default
            end
            rng(obj.seed);
            % init delay buffer with zeros
            obj.delayBuffer = zeros(1, max(1,obj.delaySteps));
            obj.lpfState = 0.0;
            obj.prevMeas = 0.0;
            obj.prevError = 0.0;
            % Conservative integral clamps: +/- torque / Ki if Ki>0
            if obj.Ki > 0
                bound = obj.torqueLimit / obj.Ki;
                obj.Imin = -bound;
                obj.Imax = +bound;
            end
        end

        function [estState, obj, torqueCmd] = getStateEstimate(obj, state, stateDot, time, log)
            % getStateEstimate  Called by Flight each cycle.
            %   Inputs:
            %     state      - true/full state struct (must contain roll rate w (rad/s))
            %     stateDot   - state derivative (not used here)
            %     time       - current time (s)
            %     log        - optional struct for logging (returned appended)
            %
            %   Outputs:
            %     estState   - state-like struct with noisy/filtered/delayed roll rate
            %     obj        - updated object (integrator/filter/buffer)
            %     torqueCmd  - torque command after PID and clamping (Nm)

            if nargin < 5, log = struct(); end
            % read true IMU measurement (roll rate). Expect state.w (rad/s)
            if isstruct(state) && isfield(state,'w')
                w_true = state.w;
            elseif isnumeric(state) && numel(state)>=1
                w_true = state(1);
            else
                error('state must contain roll rate "w"');
            end

            % optional bias random-walk update
            if obj.enableBiasRandomWalk && obj.biasDriftStd>0
                obj.imuBias = obj.imuBias + obj.biasDriftStd * sqrt(obj.sampleTime) * randn();
            end

            % inject noise + bias
            if obj.enableNoise
                meas = w_true + obj.imuBias + obj.imuNoiseStd * randn();
            else
                meas = w_true + obj.imuBias;
            end

            % first-order low-pass filter (continuous tau discretized with forward Euler)
            % LPF: x_dot = (u - x)/tau => discrete: x_{k+1} = x_k + dt*(u-x_k)/tau
            alpha = obj.sampleTime / max(obj.lpfTau, eps);
            obj.lpfState = obj.lpfState + alpha * (meas - obj.lpfState);
            filteredMeas = obj.lpfState;

            % apply delay buffer (simple FIFO)
            if obj.delaySteps <= 0
                delayedMeas = filteredMeas;
            else
                % push newest
                obj.delayBuffer = [filteredMeas, obj.delayBuffer(1:end-1)];
                delayedMeas = obj.delayBuffer(end);
            end

            % Build estimated state: copy provided state but replace w with delayedMeas
            estState = state;
            if isstruct(estState)
                estState.w = delayedMeas;
            else
                % numeric array
                estState(1) = delayedMeas;
            end

            % --- PID controller for roll rate (airframe torque) ---
            % Desired roll rate: from state command field (if present)
            if isstruct(state) && isfield(state,'w_cmd')
                w_des = state.w_cmd;
            else
                w_des = 0.0; % default desired roll rate
            end

            % current error
            e = w_des - delayedMeas;

            % derivative term: on measurement or on error
            if obj.dOnMeasurement
                % D = -Kd * (meas_dot) approximated by -Kd*(meas - prevMeas)/dt
                Dterm = - obj.Kd * (delayedMeas - obj.prevMeas) / max(obj.sampleTime, eps);
            else
                Dterm = obj.Kd * (e - obj.prevError) / max(obj.sampleTime, eps);
            end

            % raw integrator update (predictive) - we will handle anti-windup later
            I_candidate = obj.I + obj.Ki * e * obj.sampleTime;
            % clamp integrator to bounds
            I_candidate = min(max(I_candidate, obj.Imin), obj.Imax);

            % assemble raw torque (before wheel-speed derating & saturation)
            rawTorque = obj.Kp * e + I_candidate + Dterm;

            % wheel-speed based derating of torque:
            abs_w = abs(obj.wheelSpeed);
            if abs_w <= obj.omega_soft
                derate = 1.0;
            elseif abs_w >= obj.omega_max
                derate = 0.0;
            else
                % linear derating between soft and max
                derate = 1.0 - (abs_w - obj.omega_soft)/(obj.omega_max - obj.omega_soft);
            end
            torqueDerated = rawTorque * derate;

            % saturate torque command to actuator limits
            torqueCmd = min(max(torqueDerated, -obj.torqueLimit), obj.torqueLimit);

            % Anti-windup logic:
            % If raw (pre-derate) torque > torqueLimit and error > 0 => don't integrate (keep old I)
            % If raw torque < -torqueLimit and error < 0 => don't integrate
            % Otherwise accept the candidate integrator value.
            saturatedHigh = (rawTorque > obj.torqueLimit);
            saturatedLow  = (rawTorque < -obj.torqueLimit);
            if (saturatedHigh && e>0) || (saturatedLow && e<0)
                % do not update integrator -> keep previous obj.I
                % (prevents windup while actuator saturated)
            else
                obj.I = I_candidate;
            end

            % Save previous values for next step
            obj.prevError = e;
            obj.prevMeas = delayedMeas;

            % Optional log entries
            log.meas = meas;
            log.filtered = filteredMeas;
            log.delayed = delayedMeas;
            log.rawTorque = rawTorque;
            log.torqueDerated = torqueDerated;
            log.torqueCmd = torqueCmd;
            log.integrator = obj.I;
            log.derate = derate;
            if nargout>=2
                % return log in estState if caller wants combined info
                if isstruct(estState)
                    estState.navLog = log;
                end
            end
        end

        function obj = setWheelSpeed(obj, w)
            % setWheelSpeed  Update measured wheel speed (encoder). Use before calling getStateEstimate.
            obj.wheelSpeed = w;
        end

        function obj = resetIntegrator(obj, v)
            if nargin<2, v = 0.0; end
            obj.I = v;
        end
    end
end
