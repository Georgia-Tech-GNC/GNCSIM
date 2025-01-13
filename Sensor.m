classdef Sensor < handle

    properties
        q;
    end

    methods

        function obj = Sensor()
                % Constructor here             
        end

        %{
            void GPS2Flat(float lat, float lon, float alt, float* x_flat, float* y_flat, float* z_flat, float* xr, float* yr, float* zr) {
                // Converts from WGS84 to ECEF
                // Define intermediate variables
                double WGS84_E = 0.08181;
                // Convert latitude and longitude from degrees to radians
                double clat = cos(lat * (3.14 / 180.0));
                double slat = sin(lat * (3.14 / 180.0));
                double clon = cos(lon * (3.14 / 180.0));
                double slon = sin(lon * (3.14 / 180.0));
                // Convert LLA to ECEF
                double N = WGS84_A / sqrt(1.0 - WGS84_E * WGS84_E * slat * slat);
                double x_ecef = (N + alt) * clat * clon;
                double y_ecef = (N + alt) * clat * slon;
                double z_ecef = (N * (1.0 - WGS84_E * WGS84_E) + alt) * slat;
                // Convert ECEF to ENU
                // xr, yr, zr - ECEF coordinates of your ENU origin
                // x_ecef, y_ecef, z_ecef - rocket's position in ENU
                double dx = x_ecef - *xr;
                double dy = y_ecef - *yr;
                double dz = z_ecef - *zr;
                double x_enu = -slon*dx  + clon*dy;
                double y_enu = -slat*clon*dx - slat*slon*dy + clat*dz;
                double z_enu = clat*clon*dx + clat*slon*dy + slat*dz;
                // Convert ENU to FlatEarth
                *x_flat = z_enu;
                *y_flat = y_enu;
                *z_flat = -1 * x_enu;
            }
        %}


        function [x_flat, y_flat, z_flat] = GPS2Flat(lat, lon, alt, xr, yr, zr)
            
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
            N = WGS84_A / sqrt(1 - (WGS84_E * slat)^2);

            x_ecef = (N + alt) * clat * clon;
            y_ecef = (N + alt) * clat * slon;
            z_ecef = (N * (1.0 - WGS84_E * WGS84_E) + alt) * slat;

            % Convert ECEF to ENU
            % xr, yr, zr - ECEF coordinates of your ENU origin
            % x_ecef, y_ecef, z_ecef - rocket's position in ENU
            dx = x_ecef - xr;
            dy = y_ecef - yr;
            dz = z_ecef - zr;

            x_enu = -slon*dx  + clon*dy;
            y_enu = -slat*clon*dx - slat*slon*dy + clat*dz;
            z_enu = clat*clon*dx + clat*slon*dy + slat*dz;

            x_flat = z_enu; 
            y_flat = y_enu;
            z_flat = -1 * x_enu;
                
        end
        
    end

end