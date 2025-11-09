classdef ReactionWheel < ActuatorModel
    properties
        location % Location of reaction wheel on rocket
        q_ba % Quarternion of Actuator wrt Body Frame
        inertia % Inertia Tensor
        torqueLimit % Torque limit of actuator
    end

    methods 

        function obj = ReactionWheel(location, q_ba, inertia, torqueLimit)
            obj.location = location;
            obj.q_ba = q_ba;
            obj.inertia = inertia;
            obj.torqueLimit = torqueLimit;
        end
        
        function [F_b, M_b] = get_forces_and_moments(obj, control_output, rocket, CG)
            % Calculate forces and moments based on control output
            torque = min(control_output, obj.torqueLimit);
            M_b = cross([torque 0 0], obj.inertia);
            F_b = cross(obj.location - CG, M_b); % Force calculation
        end
    end
end