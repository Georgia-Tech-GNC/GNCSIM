classdef (Abstract) Sensor < handle
    properties (Abstract)
        name
        outputDimension
        noiseCovariance
        outputRate          % Hz - how often sensor outputs data
        outputProbability   % 0-1 - probability of successful output
        observationMatrix   % maps 13 states to sensor output
    end
   
    properties (Abstract, Constant)
        SENSOR_TYPE navigation_pkg.SensorType
    end

    properties
        % Set during instantiation
        outputPeriod        % seconds - time between outputs
        lastOutputTime      % last time sensor provided output
        nextOutputTime      % next scheduled output time
    end 
    
    methods
        % Constructor - child classes must initialize outputPeriod, lastOutputTime, nextOutputTime
        function obj = Sensor()
            % Empty constructor - subclasses handle initialization
        end
        
        function tf = isOutputReady(obj, currentTime)
            % Check if sensor should output at current time
            tf = (currentTime >= obj.nextOutputTime) && (rand() < (obj.outputProbability));
        end
        
        function scheduleNextOutput(obj, currentTime)
            % Update timing for next output
            obj.lastOutputTime = currentTime;
            obj.nextOutputTime = currentTime + obj.outputPeriod;
        end
        
        function setOutputRate(obj, newRate)
            obj.outputRate = newRate;
            obj.outputPeriod = 1 / newRate;
        end
        
        function [H,R] = getObservationMatrices(obj)
            H = obj.observationMatrix;
            R = obj.noiseCovariance;
        end
    end
    
    methods (Abstract)
        measurement = getMeasurement(obj, state, stateDot, currentTime)
    end
end