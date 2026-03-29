% Motor class encapsulating all rocket motor/propulsion properties and behavior.
% Main Authors: Karsten Caillet (kcaillet3@gatech.edu)

classdef Motor < handle & matlab.mixin.Copyable

    properties
        thrustCurve  (:,2) double
        totalImpulse (1,1) double {mustBeNonnegative}
        burnTime     (1,1) double {mustBeNonnegative}   % total burn time of the motor
        tIgnition    (1,1) double {mustBeNonnegative}   % time of motor ignition in the simulation
        motorMass    (1,1) double {mustBeNonnegative}   % total propellant mass (wet - dry)
        useThrust    (1,1) logical = true               % flag to enable/disable thrust 
    end

    methods
        function obj = Motor(thrustCurve, totalImpulse, burnTime, tIgnition, motorMass)
            % Motor  Construct a Motor from individual propulsion parameters.
            arguments
                thrustCurve  
                totalImpulse 
                burnTime    
                tIgnition    
                motorMass    
            end

            obj.thrustCurve  = thrustCurve;
            obj.totalImpulse = totalImpulse;
            obj.burnTime     = burnTime;
            obj.tIgnition    = tIgnition;
            obj.motorMass    = motorMass;
        end

        function currentThrust = getThrust(obj, t)
            % getThrust  Return scalar interpolated thrust [N] at simulation time t.
            arguments
                obj
                t (1,1) double {mustBeNonnegative}
            end

            if ~isnumeric(t)
                error('Error: Time must be numeric.')
            end

            timeProp = t - obj.tIgnition;

            if ~obj.useThrust || t < obj.tIgnition || timeProp > obj.burnTime
                currentThrust = 0;
                return
            end

            % Extract time and thrust columns
            tc     = obj.thrustCurve;
            time   = tc(:, 1);
            thrust = tc(:, 2);

            % Linear interpolation
            currentThrust = interp1(time, thrust, timeProp, 'linear', 0);
        end

        function mDot = getMDot(obj, time)
            % getMDot  Return propellant mass flow rate [kg/s] (negative = mass loss).
            arguments
                obj
                time (1,1) double {mustBeNonnegative}
            end
            T    = obj.getThrust(time);
            mDot = -T * obj.motorMass / obj.totalImpulse;
        end
    end
end
