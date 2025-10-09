
classdef RollPID

    properties
        Kp % Proportional gain
        Ki % Integral gain
        Kd % Derivative gain
        Kb % Back calculation gain
        Kf % Feed Forward gain
        referenceRoll % Target roll rate
        integral % Integral term
        previousError % Previous error value
        previousTime % Previous error time
        minTorque % Min torque of reaction wheel
        maxTorque % Max torque of reaction wheel
    end

    methods

        % Controller Definition
        function obj = RollPID(Kp, Ki, Kd, Kf, Kb, minTorque, maxTorque, reference_roll)
            obj.Kp = Kp;
            obj.Ki = Ki;
            obj.Kd = Kd;
            obj.Kf = Kf;
            obj.Kb = Kb; 
            obj.referenceRoll = reference_roll;
            obj.integral = 0;
            obj.previousError = 0;
            obj.previousTime = 0;
            obj.minTorque = minTorque;
            obj.maxTorque = maxTorque; 
        end

        % Return Controller Config
        function config = getConfig(obj)
            config.Kp = obj.Kp;
            config.Ki = obj.Ki;
            config.Kd = obj.Kd;
            config.Kb = obj.Kb;
            config.minTorque = obj.minTorque;
            config.maxTorque = obj.maxTorque;
            config.reference_roll = obj.referenceRoll;
        end

        % Loop Step Function
        function torque = step(obj, current_roll, desired_alpha, t)
            error = obj.referenceRoll(t) - current_roll;
            obj.integral = obj.integral + error; % Update integral term
            derivative = (error - obj.previousError) / (t - obj.previousTime); % Calculate derivative
            output = obj.Kp * error + obj.Ki * obj.integral + obj.Kd * derivative + desired_alpha(t) * obj.Kf; % PIDF output
            obj.previousError = error; % Update previous error
            obj.previousTime = t; % Update previous time
            sign = (output > 0) * 1 + (output < 0) * -1;
            torque = sign * min(max(abs(output * obj.maxTorque), obj.minTorque), obj.maxTorque); % Limit torque to maxTorque
            
            % Wind Back Logic
            if abs(output * obj.maxTorque) > obj.maxTorque
                windBack = output * obj.maxTorque - obj.maxTorque;
                obj.integral = obj.integral - windBack * obj.Kb; 
            elseif abs(output * obj.maxTorque) < obj.minTorque
                windBack = obj.minTorque - abs(output * obj.maxTorque);
                obj.integral = obj.integral - windBack * obj.Kb; 
            end
          
        end

    end
end