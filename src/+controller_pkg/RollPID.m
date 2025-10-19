classdef RollPID < Controller

    properties
        actuator % ActuatorModel
        Kp % Proportional gain
        Ki % Integral gain
        Kd % Derivative gain
        Kb % Back calculation gain
        integral % Integral term
        previousError % Previous error value
        previousTime % Previous error time
        minTorque % Min torque of reaction wheel
        maxTorque % Max torque of reaction wheel
    end

    methods

        % Constructor for RollPID class
        function obj = RollPID(actuator, Kp, Ki, Kd, Kb, Kf, minTorque, maxTorque)
            obj.actuator = actuator;
            obj.Kp = Kp;
            obj.Ki = Ki;
            obj.Kd = Kd;
            obj.Kb = Kb;
            obj.Kf = Kf;
            obj.minTorque = minTorque;
            obj.maxTorque = maxTorque;
            obj.integral = 0;
            obj.previousError = 0;
            obj.previousTime = 0;
        end

        % Set PID Gains 
        function obj = setPID(obj,Kp,Ki,Kd)
            obj.Kp = Kp;
            obj.Ki = Ki;
            obj.Kd = Kd;
        end

        % Set Torque Limits
        function obj = setTorqueLimits(obj,minTorque,maxTorque)
            obj.minTorque = minTorque;
            obj.maxTorque = maxTorque;
        end

        % Set Back Calculation Gain
        function obj = setKb(obj,Kb)
            obj.Kb = Kb; 
        end

        % Get PID Gains
        function [Kp,Ki,Kd] = getPID(obj)
            Kp = obj.Kp;
            Ki = obj.Ki;
            Kd = obj.Kd;
        end

        % Get Torque Limits
        function [minTorque, maxTorque] = getTorqueLimits(obj)
            minTorque = obj.minTorque;
            maxTorque = obj.maxTorque;
        end

        % Get Back Calculation Gain
        function [Kb] = getKb(obj)
            Kb = obj.Kb; 
        end

        function [F_b, M_b] = get_forces_and_moments(obj, state, target_state, disturbances, motor_params, CG, ~, time)
            % Roll Error
            error = target_state(6) - state(6); % Assuming roll is 6th position in state

            % Calculate the time difference
            dt = time - obj.previousTime;

            % Update integral term
            obj.integral = obj.integral + error * dt;

            % Calculate derivative term only if preivous time is not zero
            if obj.previousTime ~= 0
                derivative = (error - obj.previousError) / dt;
            else
                derivative = 0;
            end

            obj.previousError = error; % Update previous error for next iteration
            obj.previousTime = currentTime; % Update previous time for next iteration

            % Calculate the control output
            torque = obj.Kp * error + obj.Ki * obj.integral + obj.Kd * derivative;

            % Clamp the torque to the min and max limits
            clamped_torque = max(obj.minTorque, min(torque, obj.maxTorque));

            % Anti-Wind Up logic
            obj.integral = obj.integral + obj.Ki * error * obj.Kb(clamped_torque - torque);

            % Pass control output to actutator to calculate forces and moments
            % NEED TO DOUBLE CHECK INPUTS FOR FUNCTION
            [F_b, M_b] = obj.actuator.get_forces_and_moments(torque, motor_params, CG); 

        end

        % Reset Controller 
        function obj = reset(obj)
            obj.integral = 0; 
            obj.previousError = 0; 
            obj.previousTime = 0; 
        end

    end

end