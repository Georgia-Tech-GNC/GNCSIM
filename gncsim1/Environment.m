% Main Authors: Hridai Ambati
classdef Environment
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here

    properties (SetAccess = private)
        
        vm_mean = 0;  % change this for MC

        vm_3sigma = 15;  % change this for MC

        alts;

        wind_vec;

        step_size = 1;  % step size of wind vector. A step size of 10 will update the wind every 10 meters

        max_altitude = 10000;
        
    end

    methods
        function obj = Environment()
            %   Constructing Default Environment Class
            %   Assumes Default Wind Parameters as defined in MonteCarlo
            
            alpha = 0.9999;
            sigma_AR1 = (obj.vm_3sigma/3) * sqrt(1 - alpha^2);

            % wind vector (wind speed as function of altitude)
            [obj.alts, obj.wind_vec] = get_AR1(obj.vm_mean, sigma_AR1, alpha, obj.max_altitude, obj.step_size);
            % plot(obj.wind_vec, obj.alts, 'LineWidth', 1);

        end
        
        function windspeed = get_windspeed(obj, alt)
            wind_index = max(min(round(alt / obj.step_size), obj.max_altitude), 1);
            windspeed = obj.wind_vec(wind_index);
        end
        
        function [vm_mean] = get_vm_mean(obj)
            vm_mean = obj.vm_mean;
        end

        function [vm_3sigma] = get_vm_3sigma(obj)
            vm_3sigma = obj.vm_3sigma;
        end

        % Setters
        function obj = set_vm_mean(obj, vm_mean)
             obj.vm_mean = vm_mean;
        end

        function obj = set_vm_3sigma(obj, vm_3sigma)
            obj.vm_3sigma = vm_3sigma;
        end


        function [alts] = get_altitudes(obj)
            alts = obj.alts;
        end

        function [obj] = set_max_altitude(obj, altitude)
            obj.max_altitude = altitude;
        end
    end
end


function [alt_vec, wind_vec] = get_AR1(mu, sigma, alpha, max_altitude, step_size)
	alt_vec = 0 : step_size : max_altitude;
	nSteps = length(alt_vec) - 1;

	wind_vec	=	zeros(1, nSteps+1);
    wind_vec(1) = mu;  % initial
    noise		= normrnd(0, sigma, 1, nSteps+1); 

    for n = 2:nSteps+1
        wind_vec(n) = (1 - alpha)*mu + alpha*wind_vec(n-1) + noise(n);
    end
end

