classdef (Abstract) ActuatorModel
    properties
        location %used to compute moment wrt CG
        q_ba % quaternion of actuator frame wrt body frame
    end

    methods (Abstract)
        [F_b, M_b] = get_forces_and_moments(obj, states, disturbances, CG)
    end

end