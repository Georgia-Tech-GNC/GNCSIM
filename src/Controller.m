classdef (Abstract) Controller < handle
    properties
        actuator_list
    end

    methods (Abstract)
        [F_b, M_b] = get_forces_and_moments(state, disturbances, motor_params, CG, breakPoints, time);
    end
end
