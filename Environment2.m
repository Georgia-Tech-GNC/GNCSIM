% Main Authors: Hridai Ambati
classdef Environment2
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here

    properties (SetAccess = private)

        altitudes;
        
        rho;

        pressure;

        temperature;

        speed_of_sound;

        wind_speed;

        location;
        
        wind_parameters;

        index;

        vm_mean = 0;  % change this for MC

        vm_3sigma = 15;  % change this for MC

        alts;

        wind_vec;

        step_size = 1;  % step size of wind vector. A step size of 10 will update the wind every 10 meters

        max_altitude = 10000;
        
    end

    methods
        function obj = Environment2(location, use_default)
            %   Constructing Default Environment Class
            %   
            %   The location must be a 1X3 vector with the format
            %   [latitude, longitude, elevation]
            %   Requires Python in order to initialize data from rocketpy
            %
            %   Assumes Default Wind Parameters as defined in MonteCarlo

            latitude = location(1);
            longitude = location(2);
            elevation = location(3);

            % Launch Site Altitude
            obj.location = location;

            % Wind Parameters Monte Carlo Parameter Initialization
            currentDir = fileparts(mfilename('fullpath'));
            filename = fullfile(currentDir, 'Environment', 'WindParameters.xlsx');
            data = readcell(filename);
    
            % Updating Wind Parameters
            obj.wind_parameters = cell2mat(data(2:end,2:5));

            % Choosing Index
            obj.index = 3;

            if ~use_default
                % Creating Data with Rocketpy
                filename = fullfile(currentDir, 'Environment', 'generate_environment_data.py');
                [~, ~] = system(['python "' filename '" ' num2str(latitude) ' ' num2str(longitude) ' ' num2str(elevation)]);

                % Extracting Data
                filename = fullfile(currentDir, 'Environment', 'EnvironmentData.csv'); 
            else
                % Extracting Data
                filename = fullfile(currentDir, 'Environment', 'EnvironmentData_default.csv');
            end
            
            warning('off', 'all');
            data = readtable(filename);
            warning('on', 'all');

            obj.altitudes = data.Altitude';
            obj.rho = data.Density';
            obj.pressure = data.Pressure';
            obj.temperature = data.Temperature';
            obj.speed_of_sound = data.SpeedOfSound';
            %obj.wind_speed = data.WindSpeed';

            
            alpha = 0.9999;
            sigma_AR1 = (obj.vm_3sigma/3) * sqrt(1 - alpha^2);

            % wind vector (wind speed as function of altitude)
            [obj.alts, obj.wind_vec] = get_AR1(obj.vm_mean, sigma_AR1, alpha, obj.max_altitude, obj.step_size);
            plot(obj.wind_vec, obj.alts, 'LineWidth', 1);

        end
        
        function windspeed = get_windspeed(obj, alt)
            wind_index = clip(round(alt / obj.step_size), 1, obj.max_altitude);
            windspeed = obj.wind_vec(wind_index);
        end

        %{
        function wind_speed = wind_speed_change(obj, V_m, d_m, x)
            % Wind Speed Change: returns instantanous wind speed based on parameters that describe the speed and length of the gust
            % This function models a change in wind speed using a 1 -
            % cosine wave. The maximum wind speed is defined by V_m and the
            % distance traveresed into the gust before reaching the maximum
            % wind speed is d_m. The function is instantanous and returns
            % an instantaneous wind speed at specified distance moved into
            % the gust "x". After a certain distance is traversed into the
            % gust, the windspeed remains at the maximum wind speed.
            if (x < 0)
                wind_speed = 0;
            elseif (0 <= x && x < d_m)
                wind_speed = (V_m/2)*(1 - cos((pi*x)/d_m));
            else
                wind_speed = V_m;
            end
        end

        function wind_speed = wind_speed_gust(obj, V_m, d_m, x)
            % Wind Speed Change: returns instantanous wind speed based on parameters that describe the speed and length of the gust
            % This function models a change in wind speed using a symmetric 1 -
            % cosine wave. The maximum wind speed is defined by V_m and the
            % distance traveresed into the gust before reaching the maximum
            % wind speed is d_m. The function is instantanous and returns
            % an instantaneous wind speed at specified distance moved into
            % the gust "x". After a certain distance is traversed into the
            % gust, the wind speed drops down to zero.
            if (x < 0)
                wind_speed = 0;
            elseif (0 <= x && x < d_m)
                wind_speed = (V_m/2)*(1 - cos((pi*x)/d_m));
            elseif (d_m <= x && x < 2*d_m)
                wind_speed = V_m;
            elseif (2*d_m <= x && x < 3*d_m)
                wind_speed = (V_m/2)*(1 - cos(((pi*x)/d_m)+pi));
            elseif (3*d_m <= x)
                wind_speed = 0;
            end
        end

        function obj = add_wind_parameter(obj, gust_speed_mean, gust_speed_3sigma, gust_length_mean, gust_length_3sigma)
            % Adding a wind parameter (There should be 4 wind parameters added by default)
            % Specify the object, the mean and 3 times the standard
            % deviation of wind gust speed, and the mean ant he 3 times the
            % standard deviation of the wind gust length.
            % Updates object with new wind parameter
            add = [gust_speed_mean, gust_speed_3sigma, gust_length_mean, gust_length_3sigma];
            obj.wind_parameters = [obj.wind_parameters; add];
        end

        function [wind_parameters] = get_wind_parameter(obj, index)
            validateattributes(index, {'numeric'}, {'integer', 'positive', '<=', size(obj.wind_parameters, 1)}, 'get_wind_parameter', 'index');
            wind_parameters = obj.wind_parameters(index,:);
        end

        function obj = remove_wind_parameter(obj, index)
            validateattributes(index, {'numeric'}, {'integer', 'positive', '<=', size(obj.wind_parameters, 1)}, 'get_wind_parameter', 'index');
            obj.wind_parameters(index,:) = [];
        end
        
        % Getters

        function [V_m, d_m] = get_wind_aspects(obj)
            params = obj.wind_parameters(obj.index,:);

            dist1 = makedist('Normal','mu',params(1),'sigma',params(2)/3);
            dist2 = makedist('Normal','mu',params(3),'sigma',params(4)/3);

            V_m = random(dist1);
            d_m = random(dist2);
        end
        %}

        function [rho_final] = get_rho(obj, altitude_selected)
            rho_final = interp1(obj.altitudes, obj.rho, altitude_selected);
        end

        function [pressure_final] = get_pressure(obj, altitude_selected)
            pressure_final = interp1(obj.altitudes, obj.pressure, altitude_selected);
        end

        function [temperature_final] = get_temperature(obj, altitude_selected)
            temperature_final = interp1(obj.altitudes, obj.temperature, altitude_selected);
        end
            
        function [speed_of_sound_final] = get_speed_of_sound(obj, altitude_selected)
            speed_of_sound_final = interp1(obj.altitudes, obj.speed_of_sound, altitude_selected);
        end

        function [wind_speed_final] = get_wind_speed(obj, altitude_selected)
            wind_speed_final = interp1(obj.altitudes, obj.wind_speed, altitude_selected);
        end

        function [vm_mean] = get_vm_mean(obj)
            vm_mean = obj.vm_mean;
        end

        function [vm_3sigma] = get_vm_3sigma(obj)
            vm_3sigma = obj.vm_3sigma;
        end

        %function [dm_mean] = get_dm_mean(obj)
        %    dm_mean = obj.dm_mean;
        %end

        %function [dm_3sigma] = get_dm_3sigma(obj)
        %    dm_3sigma = obj.dm_3sigma;
        %end
        
        % Setters
        function obj = set_vm_mean(obj, vm_mean)
             obj.vm_mean = vm_mean;
        end

        function obj = set_vm_3sigma(obj, vm_3sigma)
            obj.vm_3sigma = vm_3sigma;
        end

        %function obj = set_dm_mean(obj, dm_mean)
        %    obj.dm_mean = dm_mean;
        %end

        %function obj = set_dm_3sigma(obj, dm_3sigma)
        %    obj.dm_3sigma = dm_3sigma;
        %end

        function [alts] = get_altitudes(obj)
            alts = obj.altitudes;
        end

        function obj = set_index(obj, value)
            validateattributes(value, {'numeric'}, {'integer', 'positive', '<=', size(obj.wind_parameters, 1)}, 'set_index', 'index');
            obj.index = value;
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

