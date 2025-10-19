classdef (Abstract) Controller < handle
    properties
        actuator
    end

    methods (Abstract)
        [F_b, M_b] = get_forces_and_moments(state, disturbances, motor_params, CG, breakPoints, time);
        gain = compute_gain()
    end
end
