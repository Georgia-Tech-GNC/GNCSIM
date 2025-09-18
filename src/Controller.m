classdef (Abstract) Controller
    properties
        actuator_list
    end

    methods (Abstract)
        [F_b, M_b] = get_forces_and_moments(obj, state, disturbances, motor_params);
    end
end
