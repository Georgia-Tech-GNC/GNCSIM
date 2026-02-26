classdef SyndromeActuatorModel < controllerPkg.ActuatorModel
    properties
        % Wheel / friction params
        Iw                 % wheel inertia
        wMax               % hard wheel speed limit [rad/s]
        b = 0              % viscous friction coefficient
        tau_c = 0          % Coulomb friction magnitude [N*m] (opposes motion)           

        % Motor / electrical params (paper Eq. voltage balance)
        Kt                 % torque constant [N*m/A]
        Ke                 % back-EMF constant [V/(rad/s)]
        R_wind             % winding resistance [ohm]
        I_max              % hardware current limit [A]

        % fixed bus voltage
        V_bus_nom          % [V]

        % Bookkeeping for applied moment
        tauApplied = 0

        % Required by framework (some code expects it exists)
        allocationMatrix = eye(3)
    end

    methods
        function obj = SyndromeActuatorModel(Iw, wMax, b, tau_c, Kt, Ke, R_wind, I_max, V_bus_nom)
            obj.Iw = Iw;
            obj.wMax = wMax;
            if nargin >= 3 && ~isempty(b), obj.b = b; end
            if nargin >= 4 && ~isempty(tau_c), obj.tau_c = tau_c; end

            obj.Kt = Kt;
            obj.Ke = Ke;
            obj.R_wind = R_wind;
            obj.I_max = I_max;

            obj.V_bus_nom = V_bus_nom;
        end

        function wdot = actuatorDynamics(obj, controlInput, actuatorState)
            % actuatorState(1) is wheel speed [rad/s]
            w = actuatorState(1);

            % Control input is body torque command about +x [N*m]
            tauCmd_body = controlInput(1);

            % Compute instantaneous true torque authority using fixed V_bus_nom (Option A)
            tauMax_true = obj.computeTauMax(w, obj.V_bus_nom);

            % Saturate body torque command to true authority
            tau_body = obj.clamp(tauCmd_body, -tauMax_true, +tauMax_true);

            % Convert to wheel torque (action-reaction)
            tauWheel = -tau_body;

            sgn_w = 0;
            if abs(w) > 1e-6
                sgn_w = sign(w);
            end
            % Wheel dynamics: Iw * wdot = tauWheel - b*w - tau_c*sgn(w)
            wdot_val = (tauWheel - obj.b * w - obj.tau_c * sgn_w) / obj.Iw;

            % Hard speed limit: prevent further acceleration past limit
            if (w >= obj.wMax && wdot_val > 0), wdot_val = 0; end
            if (w <= -obj.wMax && wdot_val < 0), wdot_val = 0; end

            wdot = zeros(size(actuatorState));
            wdot(1) = wdot_val;

            % Store what we actually applied to the body this step
            obj.tauApplied = tau_body;
        end

        function [fB, mB] = getForcesMoments(obj, ~, ~, ~, ~)
            fB = zeros(3,1);
            mB = [obj.tauApplied; 0; 0];
        end

        function tau_max = computeTauMax(obj, omega_w, V_bus)
            % Implements paper torque bound:
            % I_avail = max(0, V_bus - Ke*omega_w)/R_wind
            % I_lim   = min(I_max, I_avail)
            % tau_max = Kt * I_lim

            V_headroom = V_bus - obj.Ke * omega_w;
            I_avail = max(0.0, V_headroom) / obj.R_wind;
            I_lim = min(obj.I_max, I_avail);
            tau_max = obj.Kt * I_lim;
        end
    end

    methods (Static, Access = private)
        function y = clamp(x, lo, hi)
            y = min(max(x, lo), hi);
        end
    end
end