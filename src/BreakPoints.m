classdef BreakPoints < handle & Exportable
    % BreakPoints
    %   Holds trajectory break points for gain scheduling.
    %   Each break point stores:
    %     - Optimal state (x*)
    %     - Optimal control input (u*)
    %     - Linearized dynamics matrices (A, B)
    %     - Optimal gain(s) (K)
    %   Provides interpolation of gains between break points.

    properties (SetAccess = private)
        % Break point "progress measure" (altitude or accumulated velocity)
        progressValues double = []

        % Stored data at each break point
        states cell = {}      % {x*}
        controls cell = {}    % {u*}
        A_matrices cell = {}  % {A}
        B_matrices cell = {}  % {B}
        gains cell = {}       % {K}
    end

    methods
        function obj = BreakPoints(varargin)
            if nargin == 0
                % Empty constructor for Exportable
                return
            else
                % Later: accept trajectory data, break method, etc.
                % For now, just a placeholder
            end
        end

        function restoreFromStruct(obj, s)
            % Required for Exportable
            f = fieldnames(s);
            for i = 1:numel(f)
                try
                    obj.(f{i}) = s.(f{i});
                catch ME
                    warning('Could not restore %s: %s', f{i}, ME.message);
                end
            end
        end

        function addBreakPoint(obj, progressValue, x_opt, u_opt, A, B, K)
            % Add a new break point
            obj.progressValues(end+1) = progressValue;
            obj.states{end+1}   = x_opt;
            obj.controls{end+1} = u_opt;
            obj.A_matrices{end+1} = A;
            obj.B_matrices{end+1} = B;
            obj.gains{end+1}    = K;
        end

        function K_interp = getGain(obj, currentProgress)
            % Interpolate gain between break points
            % (placeholder: linear interpolation between nearest neighbors)
            if isempty(obj.progressValues)
                error('No break points defined.');
            end

            % Clamp if outside range
            if currentProgress <= obj.progressValues(1)
                K_interp = obj.gains{1};
                return
            elseif currentProgress >= obj.progressValues(end)
                K_interp = obj.gains{end};
                return
            end

            % Find surrounding break points
            idx = find(obj.progressValues <= currentProgress, 1, 'last');
            p1 = obj.progressValues(idx);
            p2 = obj.progressValues(idx+1);

            K1 = obj.gains{idx};
            K2 = obj.gains{idx+1};

            % Simple linear interpolation (works if K are scalars or vectors)
            alpha = (currentProgress - p1) / (p2 - p1);
            K_interp = (1-alpha)*K1 + alpha*K2;
        end
    end
end
