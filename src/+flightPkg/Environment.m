% Main Authors: Hridai Ambati (ported from GNCSIM 1.0, might need to tweak
% stuff)
classdef Environment < handle
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here

    properties (SetAccess = private)

        vmMean = 0;  % change this for MC

        vm3Sigma = 15;  % change this for MC

        alts;

        windVec;

        stepSize = 1;  % step size of wind vector. A step size of 10 will update the wind every 10 meters

        maxAltitude = 10000;

    end

    methods

        function obj = Environment()
            %   Constructing Default Environment Class
            %   Assumes Default Wind Parameters as defined in MonteCarlo

            alpha = 0.9999;
            sigmaAR1 = (obj.vm3Sigma/3) * sqrt(1 - alpha^2);

            % wind vector (wind speed as function of altitude)
            [obj.alts, obj.windVec] = getAR1(obj.vmMean, sigmaAR1, alpha, obj.maxAltitude, obj.stepSize);
            % plot(obj.windVec, obj.alts, 'LineWidth', 1);

        end

        function windspeed = getWindspeed(obj, alt)
            windIndex = max(min(round(alt / obj.stepSize), obj.maxAltitude), 1);
            windspeed = obj.windVec(windIndex);
        end

        function [vmMean] = getVmMean(obj)
            vmMean = obj.vmMean;
        end

        function [vm3Sigma] = getVm3Sigma(obj)
            vm3Sigma = obj.vm3Sigma;
        end

        % Setters
        function obj = setVmMean(obj, vmMean)
             obj.vmMean = vmMean;
        end

        function obj = setVm3Sigma(obj, vm3Sigma)
            obj.vm3Sigma = vm3Sigma;
        end


        function [alts] = getAltitudes(obj)
            alts = obj.alts;
        end

        function [obj] = setMaxAltitude(obj, altitude)
            obj.maxAltitude = altitude;
        end
    end
end


function [altVec, windVec] = getAR1(mu, sigma, alpha, maxAltitude, stepSize)
	altVec = 0 : stepSize : maxAltitude;
	nSteps = length(altVec) - 1;

	windVec	=	zeros(1, nSteps+1);
    windVec(1) = mu;  % initial
    noise		= normrnd(0, sigma, 1, nSteps+1);

    for n = 2:nSteps+1
        windVec(n) = (1 - alpha)*mu + alpha*windVec(n-1) + noise(n);
    end
end

