classdef GPS < handle

    properties
        GPS_model;
        flat_origin;
    end

    methods

        function obj = GPS(refLLA)
            
            obj.GPS_model = gpsSensor('SampleRate',100, ...
                            'ReferenceLocation',refLLA, ... % LLA of Georgia Tech
                            'PositionInputFormat','Local', ...
                            'HorizontalPositionAccuracy',0.1, ...
                            'VerticalPositionAccuracy',0.1);

            % Setting origin of flat Earth frame to be
            lat = refLLA(1);
            lon = refLLA(2);
            alt = refLLA(3);

            
            [x, y, z] = obj.LLA2ECEF(lat, lon, alt);
            obj.flat_origin = [x, y, z];

           

        end

        function [x_flat, y_flat, z_flat] = GPS2Flat(obj, lat, lon, alt, xr, yr, zr)
            
            % Convert LLA to ECEF coordinates
            [x_ecef, y_ecef, z_ecef] = obj.LLA2ECEF(lat, lon, alt);

            % Convert ECEF to ENU
            % xr, yr, zr - ECEF coordinates of your ENU origin
            % x_ecef, y_ecef, z_ecef - rocket's position in ENU
            dx = x_ecef - xr;
            dy = y_ecef - yr;
            dz = z_ecef - zr;

            % Convert latitude and longitude from degrees to radians
            clat = cosd(lat);
            slat = sind(lat);
            clon = cosd(lon);
            slon = sind(lon);

            x_enu = -slon.*dx  + clon.*dy;
            y_enu = -slat.*clon.*dx - slat.*slon.*dy + clat.*dz;
            z_enu = clat.*clon.*dx + clat.*slon.*dy + slat.*dz;

            x_flat = z_enu; 
            y_flat = y_enu;
            z_flat = -1 .* x_enu;
                
        end

        function [x_ecef, y_ecef, z_ecef] = LLA2ECEF(obj, lat, lon, alt)
            % Converts from WGS84 to ECEF
            % Define intermediate variables
            WGS84_E = 0.08181;

            % Convert latitude and longitude from degrees to radians
            clat = cosd(lat);
            slat = sind(lat);
            clon = cosd(lon);
            slon = sind(lon);

            % Convert LLA to ECEF
            WGS84_A = 6378137.0;
            N = WGS84_A ./ sqrt(1 - (WGS84_E .* slat).^2);

            x_ecef = (N + alt) .* clat .* clon;
            y_ecef = (N + alt) .* clat .* slon;
            z_ecef = (N .* (1.0 - WGS84_E .* WGS84_E) + alt) .* slat;
        end

        function [flat_state] = readGPS(obj,posState, velState)
            origin = obj.flat_origin;

            xr = origin(1);
            yr = origin(2);
            zr = origin(3);

            [lla] = obj.GPS_model(posState, velState);
            [x_flat, y_flat, z_flat] = obj.GPS2Flat(lla(:,1), lla(:,2), lla(:,3), xr, yr, zr);
            flat_state = [x_flat, y_flat, z_flat];
            
        end
        
    end

end