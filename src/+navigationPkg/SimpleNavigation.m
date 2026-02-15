classdef SimpleNavigation < handle
    % SimpleNavigation
    % GNCSIM2.0-compatible navigation module (no filtering).
    % Uses sensor objects (Gyroscope) for noise/bias; returns 14x1 estimate.

    properties
        idx = StateIndices
        gyroscope = []
        accelerometer = []  %#ok<NASGU>
    end

    methods
        function obj = SimpleNavigation(dt, gyroscope, accelerometer) %#ok<INUSD>
            if nargin >= 2, obj.gyroscope = gyroscope; end
            if nargin >= 3, obj.accelerometer = accelerometer; end
        end

        function estState = getStateEstimate(obj, state, stateDot, time, log) %#ok<INUSD>
            estState = state;

            if ~isempty(obj.gyroscope)
                omegaMeas = obj.gyroscope.getMeasurement(state, stateDot, time);
                omegaMeas = omegaMeas(:); % enforce column
                estState(obj.idx.omega_x) = omegaMeas(1);
            end
        end
    end
end