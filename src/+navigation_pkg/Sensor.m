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
        SENSOR_TYPE SensorType
    end

    properties
        % Set during instantiation
        outputPeriod        % seconds - time between outputs
        lastOutputTime      % last time sensor provided output
        nextOutputTime      % next scheduled output time
    end 
    
    methods
        % Constructor
        function obj = Sensor()
            obj.outputPeriod = 1 / outputRate;
            obj.lastOutputTime = 0;
            obj.nextOutputTime = obj.outputPeriod; % First output after one period
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
        
        function out = get_observationMatrix(obj)
            out = obj.observationMatrix;
        end
    end
    
    methods (Abstract)
        measurement = getMeasurement(obj, state, stateDot, currentTime)
    end
end