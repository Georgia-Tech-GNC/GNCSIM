classdef Environment
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here

    properties (SetAccess = private)
        rho;
        launchSiteAltitude;
        windParameters;
    end

    methods
        function obj = Environment()
            %UNTITLED3 Construct an instance of this class
            %   Detailed explanation goes here
        end

        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
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
            elseif (0 <= x && x < dm)
                wind_speed = (Vm/2)*(1 - cos((pi*x)/dm));
            elseif (dm <= x)
                wind_speed = Vm;
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
                wind_speed = Vm;
            elseif (2*d_m <= x && x < 3*d_m)
                wind_speed = (V_m/2)*(1 - cos(((pi*x)/d_m)+pi));
            elseif (3*d_m <= x)
                wind_speed = 0;
            end
        end
    end
end