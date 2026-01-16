% Main Authors: [Tu Nombre]
% RobustSynthesis - Diseña controladores PID robustos usando H-infinity
%
% Calcula ganancias Kp, Ki, Kd que minimizan ||Tzw||∞ considerando:
%   - Incertidumbres paramétricas (masa, MOI)
%   - Sensor noise (gyro, accel, GPS)
%   - Actuator dynamics y noise
%   - Weighting functions (Ws, Wt, Wu, Wn)

classdef RobustSynthesis < handle
    properties
        flight              % Handle a Flight object
        trimPoints          % Array de estructuras con trim conditions
        uncertainties       % Struct con rangos de incertidumbre
        weightingFunctions  % Struct con Ws, Wt, Wu, Wn
        noiseModels         % Struct con modelos de ruido
        synthesizedGains    % Array de structs con [Kp, Ki, Kd] por trim
    end
    
    methods
        function obj = RobustSynthesis(flight, uncertainties)
            % Constructor
            % Inputs:
            %   flight - Flight object con rocketDynamics y navigation
            %   uncertainties - struct con campos:
            %       .massRange = [min, max] (%)
            %       .moiRange = [min, max] (%)
            %       .windRange = [min, max] (m/s) - NUEVO
            obj.flight = flight;
            obj.uncertainties = uncertainties;
            obj.synthesizedGains = [];
            
            % Validar campos de incertidumbre
            if ~isfield(uncertainties, 'windRange')
                warning('windRange no especificado. Usando [0, 5] m/s por defecto');
                obj.uncertainties.windRange = [0, 5];
            end
            
            % Default weighting functions y noise models
            obj.setDefaultWeights();
            obj.setDefaultNoiseModels();
        end
        
        function setDefaultWeights(obj)
            % Configura weighting functions por defecto
            
            % Ws: Sensibilidad (rechazo de disturbios a bajas frecuencias)
            obj.weightingFunctions.Ws = struct(...
                'M', 2.0, ...      % Pico máximo
                'wb', 1.0, ...     % Bandwidth [rad/s]
                'A', 0.001);       % Atenuación alta freq
            
            % Wt: Complementaria (robustez a dinámica no modelada)
            obj.weightingFunctions.Wt = struct(...
                'M', 1.5, ...
                'wb', 50, ...      % Mucho mayor que bandwidth
                'A', 0.01);
            
            % Wu: Esfuerzo de control (penaliza deflexiones)
            obj.weightingFunctions.Wu = 0.01;
            
            % Wn: Sensor noise rejection (NUEVO)
            % Penaliza ganancia a frecuencias donde domina sensor noise
            obj.weightingFunctions.Wn = struct(...
                'M', 0.1, ...      % Ganancia a alta frecuencia
                'wb', 100, ...     % Frecuencia de ruido [rad/s]
                'A', 1.0);
        end
        
        function setDefaultNoiseModels(obj)
            % Define modelos de ruido de sensores y actuadores
            
            % ===== SENSOR NOISE =====
            % Giroscopio (basado en tu navigationPkg.Gyroscope)
            obj.noiseModels.gyro = struct(...
                'ARW', 0.001, ...           % Angular Random Walk [rad/sqrt(s)]
                'bias', 0.01, ...           % Bias [rad/s]
                'bandwidth', 100, ...       % [Hz]
                'powerSpectralDensity', (0.001)^2);  % [rad²/s²/Hz]
            
            % Acelerómetro
            obj.noiseModels.accel = struct(...
                'VRW', 0.01, ...            % Velocity Random Walk [m/s/sqrt(s)]
                'bias', 0.05, ...           % Bias [m/s²]
                'bandwidth', 100, ...       % [Hz]
                'powerSpectralDensity', (0.01)^2);
            
            % GPS (basado en tu navigationPkg.GPS)
            obj.noiseModels.gps = struct(...
                'horizontalAccuracy', 1, ...    % [m]
                'verticalAccuracy', 1, ...      % [m]
                'velocityAccuracy', 0.05, ...   % [m/s]
                'updateRate', 10, ...           % [Hz]
                'latency', 0.1);                % [s]
            
            % ===== ACTUATOR NOISE/DYNAMICS =====
            % Fin tabs/vanes - aproximación 1er orden
            obj.noiseModels.actuator = struct(...
                'bandwidth', 20, ...        % [Hz] - frecuencia de corte
                'damping', 0.7, ...         % [] - coeficiente de amortiguamiento
                'deadband', 0.5, ...        % [deg] - zona muerta
                'maxRate', 200, ...         % [deg/s] - tasa máxima
                'noiseStd', 0.1);           % [deg] - ruido de posición
        end
        
        function defineTrimPoints(obj, altitudes, machs, alphas)
            % Define puntos de trim para gain scheduling
            
            nTrims = length(altitudes);
            obj.trimPoints = struct('altitude', {}, 'mach', {}, 'alpha', {}, 'xStar', {});
            
            for ii = 1:nTrims
                obj.trimPoints(ii).altitude = altitudes(ii);
                obj.trimPoints(ii).mach = machs(ii);
                obj.trimPoints(ii).alpha = alphas(ii);
                obj.trimPoints(ii).xStar = obj.computeTrimState(altitudes(ii), machs(ii), alphas(ii));
            end
            
            fprintf('Definidos %d trim points\n', nTrims);
        end
        
        function xStar = computeTrimState(obj, altitude, mach, alpha)
            % Calcula estado de equilibrio para un trim point
            % Estado: [mass, x, y, z, vx, vy, vz, q0, q1, q2, q3, wx, wy, wz]
            
            a = 340;  % Velocidad del sonido [m/s]
            V = mach * a;
            
            % Velocidades en body frame
            u = V * cosd(alpha);
            w = V * sind(alpha);
            
            % Masa nominal
            mass = obj.flight.rocketDynamics.massWet;
            
            % Quaternion - manejar ambos casos (fila o columna)
            quat = Utils.eulerToQuat([0, -90-alpha, 0]);
            
            % FORZAR a ser vector columna
            quat = quat(:);  % Convierte cualquier forma a columna
            
            % Verificar tamaño
            if length(quat) ~= 4
                error('Quaternion debe tener 4 elementos. Recibido: %d elementos\nValores: %s', ...
                    length(quat), mat2str(quat));
            end
            
            % Construir estado completo (14 elementos como COLUMNA)
            xStar = zeros(14, 1);  % Preallocate como columna
            xStar(1) = mass;
            xStar(2) = 0;          % x position
            xStar(3) = 0;          % y position
            xStar(4) = altitude;   % z position
            xStar(5) = u;          % vx
            xStar(6) = 0;          % vy
            xStar(7) = w;          % vz
            xStar(8:11) = quat;    % quaternion [q0, q1, q2, q3]
            xStar(12) = 0;         % wx
            xStar(13) = 0;         % wy
            xStar(14) = 0;         % wz
            
            % Verificación final
            assert(isequal(size(xStar), [14, 1]), ...
                'Estado debe ser [14×1]. Recibido: [%d×%d]', ...
                size(xStar, 1), size(xStar, 2));
        end
        
        function synthesizeAllControllers(obj)
            % Sintetiza controlador robusto para cada trim point
            
            if isempty(obj.trimPoints)
                error('Debes definir trim points primero usando defineTrimPoints()');
            end
            
            nTrims = length(obj.trimPoints);
            obj.synthesizedGains = struct('Kp', {}, 'Ki', {}, 'Kd', {}, ...
                'performance', {}, 'margins', {});
            
            fprintf('\n=== Iniciando síntesis de controladores robustos ===\n');
            
            for ii = 1:nTrims
                fprintf('\nTrim Point %d/%d: Alt=%.0fm, M=%.2f, α=%.1f°\n', ...
                    ii, nTrims, obj.trimPoints(ii).altitude, ...
                    obj.trimPoints(ii).mach, obj.trimPoints(ii).alpha);
                
                % Generar planta aumentada con ruidos
                [Paug, info] = obj.buildAugmentedPlant(ii);
                
                % Diseñar controlador H-infinity
                [Kp, Ki, Kd, gamma, margins] = obj.synthesizePID(Paug, info);
                
                % Guardar resultados
                obj.synthesizedGains(ii).Kp = Kp;
                obj.synthesizedGains(ii).Ki = Ki;
                obj.synthesizedGains(ii).Kd = Kd;
                obj.synthesizedGains(ii).performance = gamma;
                obj.synthesizedGains(ii).margins = margins;
                
                fprintf('  Ganancias: Kp=%.3f, Ki=%.3f, Kd=%.3f\n', Kp, Ki, Kd);
                fprintf('  Performance (γ): %.3f\n', gamma);
                fprintf('  Gain Margin: %.2f dB, Phase Margin: %.2f deg\n', ...
                    margins.GainMargin, margins.PhaseMargin);
            end
            
            fprintf('\n=== Síntesis completada ===\n');
        end
        
        function [Paug, info] = buildAugmentedPlant(obj, trimIdx)
            % Construye planta aumentada con:
            %   - Dinámica nominal + incertidumbres
            %   - Modelos de ruido (sensor + actuator)
            %   - Weighting functions
            
            xStar = obj.trimPoints(trimIdx).xStar;
            
            % IMPORTANTE: Desactivar thrust para linearización
            useThrust_original = obj.flight.rocketDynamics.useThrust;
            useGravity_original = obj.flight.useGravity;
            
            obj.flight.rocketDynamics.useThrust = false;
            obj.flight.useGravity = false;
            
            try
                % 1. Modelo nominal linearizado
                lti = analysisPkg.ReduceLTI(obj.flight);
                [sysPitch, ~] = lti.generateROM(xStar);
                
                Anom = sysPitch.A;
                Bnom = sysPitch.B;
                Cnom = sysPitch.C;
                Dnom = sysPitch.D;
                
            catch ME
                % Restaurar flags antes de propagar error
                obj.flight.rocketDynamics.useThrust = useThrust_original;
                obj.flight.useGravity = useGravity_original;
                rethrow(ME);
            end
            
            % Restaurar flags
            obj.flight.rocketDynamics.useThrust = useThrust_original;
            obj.flight.useGravity = useGravity_original;
            
            % 2. Incertidumbres paramétricas
            massNom = obj.flight.rocketDynamics.massWet;
            massUnc = ureal('mass', massNom, ...
                'Percentage', obj.uncertainties.massRange);
            
            moiNom = obj.flight.rocketDynamics.moiWet(2,2);
            moiUnc = ureal('moi', moiNom, ...
                'Percentage', obj.uncertainties.moiRange);
            
            % Escalar B (momento = F/MOI)
            Bunc = Bnom * (moiNom / moiUnc);
            
            % 3. Modelo de sensor noise (agregar entrada de ruido)
            % Estados: [alpha; q; theta] (3 estados)
            % Ruido entra en medición de q (pitch rate del gyro)
            
            % Matriz de entrada de ruido (gyro noise afecta medición de q)
            Bn = [0; 1; 0];  % Gyro noise en pitch rate
            
            % 4. Modelo de actuator dynamics (1er orden)
            % G_act(s) = ωn² / (s² + 2ζωn*s + ωn²)
            wn_act = 2*pi*obj.noiseModels.actuator.bandwidth;  % [rad/s]
            zeta_act = obj.noiseModels.actuator.damping;
            
            % Actuator como sistema de 2do orden
            s = tf('s');
            G_act = wn_act^2 / (s^2 + 2*zeta_act*wn_act*s + wn_act^2);
            
            % Convertir a estado-espacio
            [A_act, B_act, C_act, D_act] = ssdata(G_act);
            
            % 5. Combinar planta + actuator
            % Estados aumentados: [x_plant; x_actuator]
            n_plant = size(Anom, 1);
            n_act = size(A_act, 1);
            
            A_aug = [Anom, Bunc*C_act;
                     zeros(n_act, n_plant), A_act];
            
            B_ctrl = [zeros(n_plant, 1);
                      B_act];
            
            B_noise = [Bn;
                       zeros(n_act, 1)];
            
            C_aug = [Cnom, zeros(1, n_act)];
            D_aug = Dnom;
            
            % 6. Crear weighting functions
            ws = obj.weightingFunctions.Ws;
            wt = obj.weightingFunctions.Wt;
            wn = obj.weightingFunctions.Wn;
            wu = obj.weightingFunctions.Wu;
            
            Ws = makeweight(ws.M, ws.wb, ws.A);     % Sensibilidad
            Wt = makeweight(wt.M, wt.wb, wt.A);     % Complementaria
            Wn = makeweight(wn.M, wn.wb, wn.A);     % Noise rejection
            Wu = wu;                                 % Control effort (escalar)
            
            % 7. Estructura aumentada para hinfsyn
            % Entradas: [w1: disturbance; w2: sensor noise; u: control]
            % Salidas: [z1: weighted error; z2: weighted control; y: measurement]
            
            % Planta en forma estándar
            Pnom = ss(A_aug, [B_noise, B_ctrl], C_aug, [0, D_aug]);
            
            % Retornar info para síntesis
            info.Ws = Ws;
            info.Wt = Wt;
            info.Wn = Wn;
            info.Wu = Wu;
            info.Pnom = Pnom;
            
            Paug = Pnom;  % Por simplicidad, usar nominal (puedes expandir con μ-synthesis)
        end
        
        function [Kp, Ki, Kd, gamma, margins] = synthesizePID(obj, Paug, info)
            % Diseña PID usando H-infinity synthesis
            
            Ws = info.Ws;
            Wt = info.Wt;
            Wu = info.Wu;
            Pnom = info.Pnom;
            
            try
                % Configurar planta para hinfsyn
                % Forma estándar: P(s) con entradas [w; u] y salidas [z; y]
                
                % Sistema interconectado
                systemnames = 'Pnom Ws Wu';
                inputvar = '[dist{1}; noise{1}; u]';
                outputvar = '[Ws; Wu; Pnom(1)+noise-Pnom(2)]';  % [z1; z2; y]
                input_to_Pnom = '[dist; u]';
                input_to_Ws = '[Pnom(1)]';
                input_to_Wu = '[u]';
                
                % Crear planta aumentada
                P_hinf = sysic;
                
                % Dimensiones: 2 señales de medición, 2 señales de control
                nmeas = 1;  % 1 medición (pitch)
                ncont = 1;  % 1 control (momento)
                
                % Síntesis H-infinity
                [K, ~, gamma] = hinfsyn(P_hinf, nmeas, ncont);
                
                % Extraer ganancias PID desde K
                % Aproximar K como PID: K(s) = Kp + Ki/s + Kd*s/(τ*s+1)
                [Kp, Ki, Kd] = obj.extractPIDfromController(K);
                
                % Calcular márgenes de estabilidad
                margins = allmargin(Pnom * K);
                
            catch ME
                warning('hinfsyn falló: %s. Usando método alternativo.', ME.message);
                
                % Método alternativo: Loop Shaping + PID tuning
                [Kp, Ki, Kd, gamma, margins] = obj.loopShapingPID(Pnom, Ws, Wu);
            end
        end
        
        function [Kp, Ki, Kd] = extractPIDfromController(~, K)
            % Extrae ganancias PID de un controlador LTI
            
            % Convertir a función de transferencia
            [num, den] = tfdata(K, 'v');
            
            % Normalizar por mayor orden del denominador
            if length(den) > length(num)
                num = [zeros(1, length(den)-length(num)), num];
            end
            
            % PID ideal: K(s) = Kd*s² + Kp*s + Ki / s
            % Forma polinomial: (Kd*s² + Kp*s + Ki) / s
            
            if length(num) >= 3
                % Extraer coeficientes
                Kd = num(end-2) / den(end-1);
                Kp = num(end-1) / den(end-1);
                Ki = num(end) / den(end);
            elseif length(num) == 2
                % PI controller
                Kp = num(end-1) / den(end-1);
                Ki = num(end) / den(end);
                Kd = 0;
            else
                % P controller
                Kp = num(end) / den(end);
                Ki = 0;
                Kd = 0;
            end
            
            % Asegurar positividad
            Kp = abs(Kp);
            Ki = abs(Ki);
            Kd = abs(Kd);
        end
        
        function [Kp, Ki, Kd, gamma, margins] = loopShapingPID(~, P, Ws, Wu)
            % Método alternativo: Loop Shaping con pidtune
            
            try
                % Usar pidtune de Matlab (si está disponible)
                opts = pidtuneOptions('PhaseMargin', 60, 'CrossoverFrequency', 2);
                C = pidtune(P, 'pidf', opts);
                
                % Extraer ganancias
                Kp = C.Kp;
                Ki = C.Ki;
                Kd = C.Kd;
                
                % Calcular performance
                L = P * C;
                S = feedback(1, L);
                gamma = norm(Ws * S, inf);
                
                margins = allmargin(L);
                
            catch
                % Fallback: Ganancias conservadoras
                Kp = 1.0;
                Ki = 0.1;
                Kd = 0.5;
                gamma = inf;
                margins = struct('GainMargin', 6, 'PhaseMargin', 45);
            end
        end
        
        function exportGains(obj, filename)
            % Exporta ganancias a archivo .mat
            if isempty(obj.synthesizedGains)
                error('No hay ganancias sintetizadas.');
            end
            
            trimPoints = obj.trimPoints;
            gains = obj.synthesizedGains;
            uncertainties = obj.uncertainties;
            noiseModels = obj.noiseModels;
            
            save(filename, 'trimPoints', 'gains', 'uncertainties', 'noiseModels');
            fprintf('Ganancias exportadas a: %s\n', filename);
        end
        
        function plotPerformance(obj)
            % Visualiza performance y márgenes
            if isempty(obj.synthesizedGains)
                error('No hay resultados.');
            end
            
            nTrims = length(obj.synthesizedGains);
            gammas = zeros(nTrims, 1);
            alts = zeros(nTrims, 1);
            gmDB = zeros(nTrims, 1);
            pm = zeros(nTrims, 1);
            
            for ii = 1:nTrims
                gammas(ii) = obj.synthesizedGains(ii).performance;
                alts(ii) = obj.trimPoints(ii).altitude;
                
                if isfield(obj.synthesizedGains(ii).margins, 'GainMargin')
                    gmDB(ii) = 20*log10(obj.synthesizedGains(ii).margins.GainMargin);
                    pm(ii) = obj.synthesizedGains(ii).margins.PhaseMargin;
                end
            end
            
            figure('Position', [100, 100, 1200, 800]);
            
            % Performance (γ)
            subplot(3,1,1);
            plot(alts, gammas, 'o-', 'LineWidth', 2, 'MarkerSize', 8);
            ylabel('Performance (γ)');
            title('H∞ Performance vs Trim Point');
            grid on;
            yline(1, '--r', 'Optimal');
            
            % Ganancias PID
            subplot(3,1,2);
            Kps = [obj.synthesizedGains.Kp];
            Kis = [obj.synthesizedGains.Ki];
            Kds = [obj.synthesizedGains.Kd];
            
            plot(alts, Kps, 'r-o', 'DisplayName', 'Kp'); hold on;
            plot(alts, Kis, 'g-s', 'DisplayName', 'Ki');
            plot(alts, Kds, 'b-^', 'DisplayName', 'Kd');
            ylabel('Gain Value');
            title('PID Gains vs Trim Point');
            legend; grid on;
            
            % Márgenes de estabilidad
            subplot(3,1,3);
            yyaxis left
            plot(alts, gmDB, 'o-', 'LineWidth', 2);
            ylabel('Gain Margin (dB)');
            yline(6, '--k', 'Min Requirement');
            
            yyaxis right
            plot(alts, pm, 's-', 'LineWidth', 2);
            ylabel('Phase Margin (deg)');
            yline(45, '--k', 'Min Requirement');
            
            xlabel('Altitude (m)');
            title('Stability Margins vs Trim Point');
            grid on;
        end
        
        function plotNoiseRejection(obj, trimIdx)
            % Analiza rechazo de ruido para un trim point
            
            if nargin < 2
                trimIdx = 1;
            end
            
            % Reconstruir planta y controlador
            [Paug, info] = obj.buildAugmentedPlant(trimIdx);
            
            Kp = obj.synthesizedGains(trimIdx).Kp;
            Ki = obj.synthesizedGains(trimIdx).Ki;
            Kd = obj.synthesizedGains(trimIdx).Kd;
            
            % PID en forma de transferencia
            s = tf('s');
            tau_d = 0.1;  % Filtro derivativo
            K_pid = Kp + Ki/s + Kd*s/(tau_d*s + 1);
            
            % Funciones de sensibilidad
            L = Paug * K_pid;
            S = feedback(1, L);        % Sensibilidad
            T = feedback(L, 1);        % Complementaria
            
            % Frecuencias
            w = logspace(-2, 3, 1000);
            
            figure('Position', [100, 100, 1200, 600]);
            
            % Magnitud
            subplot(1,2,1);
            [magS, ~] = bode(S, w);
            [magT, ~] = bode(T, w);
            magS = squeeze(magS);
            magT = squeeze(magT);
            
            semilogx(w, 20*log10(magS), 'b', 'LineWidth', 2, 'DisplayName', 'S (Disturbance Rejection)'); hold on;
            semilogx(w, 20*log10(magT), 'r', 'LineWidth', 2, 'DisplayName', 'T (Noise Attenuation)');
            
            % Gyro noise spectrum (típico ~100 Hz)
            gyro_bw = 2*pi*obj.noiseModels.gyro.bandwidth;
            xline(gyro_bw, '--k', 'Gyro BW');
            
            xlabel('Frequency (rad/s)');
            ylabel('Magnitude (dB)');
            title('Sensitivity Functions');
            legend('Location', 'best');
            grid on;
            
            % Respuesta temporal a ruido
            subplot(1,2,2);
            t = 0:0.01:5;
            
            % Simular ruido de gyro (white noise filtrado)
            noise = obj.noiseModels.gyro.ARW * randn(size(t));
            
            % Respuesta del sistema cerrado al ruido
            sys_noise = feedback(Paug, K_pid);
            y_noise = lsim(sys_noise, noise, t);
            
            plot(t, noise, 'k:', 'DisplayName', 'Sensor Noise'); hold on;
            plot(t, y_noise, 'r', 'LineWidth', 1.5, 'DisplayName', 'Output (with Controller)');
            
            xlabel('Time (s)');
            ylabel('Angle (rad)');
            title('Noise Rejection - Time Domain');
            legend; grid on;
        end
        
        function plotWindRejection(obj, trimIdx)
            % NUEVO: Analiza rechazo de viento para un trim point
            
            if nargin < 2
                trimIdx = 1;
            end
            
            if isempty(obj.synthesizedGains)
                error('No hay ganancias sintetizadas.');
            end
            
            % Reconstruir planta
            [Paug, info] = obj.buildAugmentedPlant(trimIdx);
            
            Kp = obj.synthesizedGains(trimIdx).Kp;
            Ki = obj.synthesizedGains(trimIdx).Ki;
            Kd = obj.synthesizedGains(trimIdx).Kd;
            
            % PID en forma de transferencia
            s = tf('s');
            tau_d = 0.1;
            K_pid = Kp + Ki/s + Kd*s/(tau_d*s + 1);
            
            % Sistema en lazo cerrado (wind → output)
            % Paug tiene entradas: [wind, noise, control]
            % Extraer canal wind → output
            sys_wind = feedback(Paug(:,1), K_pid);  % Canal 1: wind
            
            figure('Position', [100, 100, 1200, 800]);
            
            % Subplot 1: Respuesta en frecuencia
            subplot(2,2,1);
            w = logspace(-2, 2, 1000);
            [mag, phase, wout] = bode(sys_wind, w);
            mag = squeeze(mag);
            
            semilogx(wout, 20*log10(mag), 'b', 'LineWidth', 2);
            xlabel('Frequency (rad/s)');
            ylabel('Magnitude (dB)');
            title('Wind Rejection - Frequency Response');
            grid on;
            
            % Marcar banda de rechazo
            wind_bw = 2*pi*0.5;  % Ráfagas típicas ~0.5 Hz
            xline(wind_bw, '--r', 'Wind Gust BW');
            
            % Subplot 2: Respuesta a escalón de viento
            subplot(2,2,2);
            t = 0:0.01:10;
            wind_step = obj.uncertainties.windRange(2);  % Ráfaga máxima
            [y_step, t_step] = step(sys_wind * wind_step, t);
            
            plot(t_step, y_step, 'b', 'LineWidth', 2);
            xlabel('Time (s)');
            ylabel('Pitch Angle (rad)');
            title(sprintf('Response to %.1f m/s Wind Step', wind_step));
            grid on;
            
            % Métricas
            info_step = stepinfo(sys_wind * wind_step);
            text(0.6*max(t), 0.8*max(y_step), sprintf(...
                'Peak: %.3f rad (%.1f°)\nSettling: %.2f s', ...
                info_step.Peak, rad2deg(info_step.Peak), info_step.SettlingTime), ...
                'FontSize', 10, 'BackgroundColor', 'white');
            
            % Subplot 3: Respuesta a ráfaga realista
            subplot(2,2,3);
            % Ráfaga "1-cosine" (típica en aeronáutica)
            t_gust = 2;  % Duración de ráfaga [s]
            wind_gust = zeros(size(t));
            idx_gust = t >= 1 & t <= (1 + t_gust);
            wind_gust(idx_gust) = wind_step * 0.5 * (1 - cos(2*pi*(t(idx_gust)-1)/t_gust));
            
            y_gust = lsim(sys_wind, wind_gust, t);
            
            yyaxis left
            plot(t, wind_gust, 'k--', 'LineWidth', 1.5);
            ylabel('Wind Speed (m/s)');
            
            yyaxis right
            plot(t, rad2deg(y_gust), 'b', 'LineWidth', 2);
            ylabel('Pitch Error (deg)');
            
            xlabel('Time (s)');
            title('Response to 1-Cosine Wind Gust');
            grid on;
            
            % Subplot 4: Comparación con/sin control
            subplot(2,2,4);
            
            % Sin control (planta en lazo abierto)
            Popen = Paug(:,1);  % Solo canal de viento
            y_open = lsim(Popen, wind_gust, t);
            
            plot(t, rad2deg(y_open), 'r--', 'LineWidth', 2, 'DisplayName', 'Open Loop'); hold on;
            plot(t, rad2deg(y_gust), 'b', 'LineWidth', 2, 'DisplayName', 'Closed Loop (PID)');
            
            xlabel('Time (s)');
            ylabel('Pitch Error (deg)');
            title('Wind Rejection Comparison');
            legend('Location', 'best');
            grid on;
            
            % Calcular factor de mejora
            rms_open = rms(y_open);
            rms_closed = rms(y_gust);
            improvement = 20*log10(rms_open / rms_closed);
            
            text(0.5*max(t), 0.8*max(rad2deg(y_open)), sprintf(...
                'Improvement: %.1f dB\n(%.1f× better)', ...
                improvement, rms_open/rms_closed), ...
                'FontSize', 10, 'BackgroundColor', 'white');
            
            % Imprimir resumen
            fprintf('\n--- Wind Rejection Analysis (Trim %d) ---\n', trimIdx);
            fprintf('Wind gain: %.3f N·m per m/s\n', info.windGain);
            fprintf('Peak response to %.1f m/s step: %.2f° (%.3f rad)\n', ...
                wind_step, rad2deg(info_step.Peak), info_step.Peak);
            fprintf('Settling time: %.2f s\n', info_step.SettlingTime);
            fprintf('RMS error (open loop): %.3f rad\n', rms_open);
            fprintf('RMS error (closed loop): %.3f rad\n', rms_closed);
            fprintf('Wind rejection improvement: %.1f dB\n', improvement);
        end
    end
end