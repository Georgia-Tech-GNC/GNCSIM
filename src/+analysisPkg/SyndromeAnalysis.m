% SyndromeAnalysis
%   Lightweight plotting utilities for Syndrome roll-rate tuning runs.
%   Supports:
%     - analysisPkg.Log objects
%     - .mat files saved via analysisPkg.Log.saveToFile(), which store a struct `data`
%
classdef SyndromeAnalysis
    methods (Static)

        function out = loadSavedLog(matFile)
            % loadSavedLog  Load a .mat file created by Log.saveToFile().
            % Returns a struct with fields matching the saved `data` layout.
            s = load(matFile);
            if isfield(s, 'data')
                out = s.data;
                return;
            end
            error("SyndromeAnalysis:BadFile", "Expected variable `data` in %s (created by Log.saveToFile).", matFile);
        end

        function plotRun(logOrData, opts)
            % plotRun  Generate core Syndrome plots for a single run.
            %
            % opts.outDir (string)  Directory to save PNGs. If empty/missing, figures display.
            % opts.tStart, opts.tEnd optional time clamp.
            % opts.prefix optional filename prefix.
            if nargin < 2, opts = struct(); end
            outDir  = SyndromeAnalysis.getOpt(opts, 'outDir', '');
            tStart  = SyndromeAnalysis.getOpt(opts, 'tStart', []);
            tEnd    = SyndromeAnalysis.getOpt(opts, 'tEnd', []);
            prefix  = SyndromeAnalysis.getOpt(opts, 'prefix', 'syndrome_');

            % Uniform accessors for Log vs saved struct
            [time, getState, getControlInput, getActState, hasActState] = SyndromeAnalysis.makeAccessors(logOrData);

            if isempty(time)
                error("SyndromeAnalysis:EmptyLog", "Log has empty time vector.");
            end

            mask = true(size(time));
            if ~isempty(tStart), mask = mask & (time >= tStart); end
            if ~isempty(tEnd),   mask = mask & (time <= tEnd);   end

            t = time(mask);

            % ---- Required signals (already exist) ----
            wx_true = getState('wx'); wx_true = wx_true(mask);
            tau_cmd = getControlInput('Roll'); tau_cmd = tau_cmd(mask);

            omega_w_true = getActState('omega_w_true'); omega_w_true = omega_w_true(mask);
            omega_w_meas = getActState('omega_w_meas'); omega_w_meas = omega_w_meas(mask);
            Vbus_true    = getActState('V_bus_true');   Vbus_true    = Vbus_true(mask);
            Vbus_meas    = getActState('V_bus_meas');   Vbus_meas    = Vbus_meas(mask);
            tau_max_meas = getActState('tau_max_meas'); tau_max_meas = tau_max_meas(mask);

            % ---- Newly recommended signals (may not exist yet) ----
            wx_meas = [];
            if hasActState('wx_meas')
                wx_meas = getActState('wx_meas'); wx_meas = wx_meas(mask);
            end

            dt_k = [];
            if hasActState('dt_k')
                dt_k = getActState('dt_k'); dt_k = dt_k(mask);
            end

            tau_dist_x = [];
            if hasActState('tau_dist_x')
                tau_dist_x = getActState('tau_dist_x'); tau_dist_x = tau_dist_x(mask);
            end

            % ----------------------------
            % Plot 1: Roll rate (truth + meas if available)
            % ----------------------------
            fig = figure('Visible', SyndromeAnalysis.figVis(outDir));
            hold on; grid on;
            plot(t, wx_true, 'LineWidth', 2);
            if ~isempty(wx_meas)
                plot(t, wx_meas, 'LineWidth', 2);
                legend('\omega_x true','\omega_x meas','Location','best');
            else
                legend('\omega_x true','Location','best');
            end
            xlabel('Time (s)');
            ylabel('\omega_x (rad/s)');
            title('Roll Rate');
            SyndromeAnalysis.maybeSave(fig, outDir, [prefix 'roll_rate.png']);

            % ----------------------------
            % Plot 2: Torque command + clamp band
            % ----------------------------
            fig = figure('Visible', SyndromeAnalysis.figVis(outDir));
            hold on; grid on;
            plot(t, tau_cmd, 'LineWidth', 2);
            plot(t,  tau_max_meas, '--', 'LineWidth', 2);
            plot(t, -tau_max_meas, '--', 'LineWidth', 2);
            legend('\tau_{cmd}', '\tau_{max}^{meas}', '-\tau_{max}^{meas}', 'Location','best');
            xlabel('Time (s)');
            ylabel('Torque (N·m)');
            title('Torque Command and Saturation Bound');
            SyndromeAnalysis.maybeSave(fig, outDir, [prefix 'torque_and_limits.png']);

            % ----------------------------
            % Plot 3: Wheel speed + Vbus
            % ----------------------------
            fig = figure('Visible', SyndromeAnalysis.figVis(outDir));
            tiledlayout(2,1);

            nexttile; hold on; grid on;
            plot(t, omega_w_true, 'LineWidth', 2);
            plot(t, omega_w_meas, 'LineWidth', 2);
            legend('\omega_w true','\omega_w meas','Location','best');
            xlabel('Time (s)'); ylabel('\omega_w (rad/s)');
            title('Reaction Wheel Speed');

            nexttile; hold on; grid on;
            plot(t, Vbus_true, 'LineWidth', 2);
            plot(t, Vbus_meas, 'LineWidth', 2);
            legend('V_{bus} true','V_{bus} meas','Location','best');
            xlabel('Time (s)'); ylabel('V_{bus} (V)');
            title('Bus Voltage');

            SyndromeAnalysis.maybeSave(gcf, outDir, [prefix 'wheel_and_vbus.png']);

            % ----------------------------
            % Plot 4: Disturbance torque (if available)
            % ----------------------------
            if ~isempty(tau_dist_x)
                fig = figure('Visible', SyndromeAnalysis.figVis(outDir));
                grid on; hold on;
                plot(t, tau_dist_x, 'LineWidth', 2);
                xlabel('Time (s)');
                ylabel('\tau_{dist,x} (N·m)');
                title('Injected Roll Disturbance Torque');
                SyndromeAnalysis.maybeSave(fig, outDir, [prefix 'disturbance_torque.png']);
            end

            % ----------------------------
            % Plot 5: dt_k (if available)
            % ----------------------------
            if ~isempty(dt_k)
                fig = figure('Visible', SyndromeAnalysis.figVis(outDir));
                grid on; hold on;
                plot(t, dt_k, 'LineWidth', 2);
                xlabel('Time (s)');
                ylabel('dt_k (s)');
                title('Variable Control/Integration Timestep');
                SyndromeAnalysis.maybeSave(fig, outDir, [prefix 'dt_k.png']);
            end
        end
    end

    methods (Static, Access=public)
        function v = getOpt(opts, field, defaultVal)
            if isstruct(opts) && isfield(opts, field)
                v = opts.(field);
            else
                v = defaultVal;
            end
        end
    end

    methods (Static, Access=private)

        function vis = figVis(outDir)
            if isempty(outDir)
                vis = 'on';
            else
                vis = 'off';
            end
        end

        function maybeSave(fig, outDir, filename)
            if isempty(outDir)
                return;
            end
            if ~exist(outDir, 'dir'), mkdir(outDir); end
            saveas(fig, fullfile(outDir, filename));
            close(fig);
        end

        function [time, getState, getControlInput, getActState, hasActState] = makeAccessors(logOrData)
            if isa(logOrData, 'analysisPkg.Log')
                log = logOrData;
                time = log.time(1:log.currentIdx);

                getState = @(name) log.getState(name);
                getControlInput = @(name) log.getControlInput(name);
                getActState = @(name) log.getActuatorState(name);
                hasActState = @(name) any(strcmp(log.actuatorStateNames, name));
                return;
            end

            if isstruct(logOrData) && isfield(logOrData,'time')
                data = logOrData;
                time = data.time(:).';

                getState = @(name) SyndromeAnalysis.structLookup(data.states, data.stateNames, name);
                getControlInput = @(name) SyndromeAnalysis.structLookup(data.controlInputs, data.controlInputNames, name);
                getActState = @(name) SyndromeAnalysis.structLookup(data.actuatorState, data.actuatorStateNames, name);
                hasActState = @(name) any(strcmp(data.actuatorStateNames, name));
                return;
            end

            error("SyndromeAnalysis:BadInput", "Expected analysisPkg.Log or a saved `data` struct from Log.saveToFile().");
        end

        function vec = structLookup(mat, names, name)
            idx = find(strcmp(names, name), 1);
            if isempty(idx)
                error("SyndromeAnalysis:MissingField", "Field '%s' not found.", name);
            end
            vec = mat(idx, :);
        end
    end
end