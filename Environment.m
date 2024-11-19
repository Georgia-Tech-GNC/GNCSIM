classdef Environment
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

    end

    methods
        function obj = Environment(location)
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

            % Creating Data with Rocketpy
            filename = fullfile(currentDir, 'Environment', 'generate_environment_data.py');
            [~, ~] = system(['python "' filename '" ' num2str(latitude) ' ' num2str(longitude) ' ' num2str(elevation)]);

            % Extracting Data
            filename = fullfile(currentDir, 'Environment', 'EnvironmentData.csv'); 

            warning('off', 'all');
            data = readtable(filename);
            warning('on', 'all');

            obj.altitudes = data.Altitude';
            obj.rho = data.Density';
            obj.pressure = data.Pressure';
            obj.temperature = data.Temperature';
            obj.speed_of_sound = data.SpeedOfSound';
            obj.wind_speed = data.WindSpeed';

        end

        function wind_speed = wind_speed_change(V_m, d_m, x)
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
            elseif (d_m <= x)
                wind_speed = V_m;
            end
        end

        function wind_speed = wind_speed_gust(V_m, d_m, x)
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
            % Deletes wind parameter at specified index
            if index > size(obj.wind_parameters, 1)
            end
            wind_parameters = obj.wind_parameters(index,:);
        end

        function obj = remove_wind_parameter(obj, index)
            obj.wind_parameters(index,:) = [];
        end

        function [V_m, d_m] = get_wind_aspects(obj, index)
            params = obj.wind_parameters(index,:);

            dist1 = makedist('Normal','mu',params(1),'sigma',params(2)/3);
            dist2 = makedist('Normal','mu',params(3),'sigma',params(4)/3);

            V_m = random(dist1);
            d_m = random(dist2);
        end

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
    end
end
