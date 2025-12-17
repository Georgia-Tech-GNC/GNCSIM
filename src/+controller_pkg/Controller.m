classdef (Abstract) Controller < handle
    properties
        actuator
    end

    methods (Abstract)
        [F_b, M_b] = get_forces_and_moments(state, CG, breakPoints, time);
    end
end
