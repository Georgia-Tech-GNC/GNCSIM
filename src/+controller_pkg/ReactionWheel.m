classdef ReactionWheel < ActuatorModel
    
    properties
        location = [0; 0; 0] % Location of reaction wheel on rocket
        q_ba = [1; 0; 0; 0] % Quarternion of Actuator wrt Body Frame
        inertia = eye(3) % Inertia Tensor
        torqueLimit = inf % Torque limit of actuator
    end

    methods 

        % Actutator Model Definition
        function obj = ReactionWheel(location, q_ba, inertia, torqueLimit)
            obj.location = location;
            obj.q_ba = q_ba;
            obj.inertia = inertia;
            obj.torqueLimit = torqueLimit;
        end

        % Set Location of Reaction Wheel on Rocket
        function setLocation(obj,location)
            obj.location = location; 
        end

        % Set Torque Limit
        function setTorqueLimit(obj, torqueLimit)
            obj.torqueLimit = torqueLimit; 
        end

        % Set Inertia Tensor
        function setInertia(obj, inertia)
            obj.inertia = inertia; 
        end

        % Set Quarternion 
        function setQ(obj,q_ba)
            obj.q_ba = q_ba;
        end

        % Get Location of Reaction Wheel on Rocket
        function location = getLocation(obj)
            location = obj.location;
        end

        % Get Torque Limit
        function torqueLimit = getTorqueLimit(obj)
            torqueLimit = obj.torqueLimit;
        end

         % Get Inertia Tensor
        function inertia = getInertia(obj)
            inertia = obj.inertia;
        end

        % Get Quarternion 
        function q_ba = getQ(obj)
            q_ba = obj.q_ba;
        end

        % Reset Actuator Model
        function obj = reset(obj)
            obj.location = [0; 0; 0];
            obj.q_ba = [1; 0; 0; 0]; 
            obj.inertia = eye(3); 
            obj.torqueLimit = inf; 
        end
        
         % Calculate forces and moments based on control output
        function [F_b, M_b] = get_forces_and_moments(obj, controlOutput, ~, CG)
            torque = min(controlOutput, obj.torqueLimit);
            M_b = [torque 0 0];
            F_b = cross(obj.location - CG, M_b);
        end

        % Get Angular Acceleration
        function alpha = get_angular_acceleration(obj, M_b)
            alpha = cross(obj.inertia, M_b);
        end

    end
end