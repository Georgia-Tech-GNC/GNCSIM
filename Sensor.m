classdef Sensor < handle

    properties
        test_accel_params;
        test_gyro_params;
        test_mag_params;
        ADIS16500_accel_params;
        ADIS_16500_gyro_params;
        LIS3MDL_params;
        IMU;
        GPS
    end

    methods

        function obj = Sensor(refLLA)

            test_accel_params = accelparams('MeasurementRange',392, ...
                                             'ConstantBias',[0 0 0], ...
                                             'NoiseDensity',[800e-6 700e-6 750e-6], ...
                                             'BiasInstability',[125.0e-4 125.0e-4 134.0e-4]);
            
            test_gyro_params = gyroparams('MeasurementRange',deg2rad(2000), ...
                                            'ConstantBias',deg2rad([0 0 0]), ...
                                            'NoiseDensity',deg2rad([6.1e-3 6.1e-3 7e-3]), ...
                                            'BiasInstability',10*deg2rad([7.5 8.1 4.9])*(1/3600), ...
                                            'AccelerationBias',deg2rad([7.5 8.1 4.9])*(1/3600));
            
            test_mag_params = magparams('MeasurementRange',G2microT(16), ...
                                       'ConstantBias',[0 0 0]);
            
            ADIS16500_accel_params = accelparams('MeasurementRange',392, ...
                                                 'ConstantBias',[58.8e-3 58.8e-3 58.8e-3], ...
                                                 'NoiseDensity',[880e-6 880e-6 732e-6], ...
                                                 'BiasInstability',[125.0e-6 125.0e-6 134.0e-6]);
            
            ADIS_16500_gyro_params = gyroparams('MeasurementRange',deg2rad(2000), ...
                                                'ConstantBias',deg2rad([0.14 1.4 0.14]), ...
                                                'NoiseDensity',deg2rad([6.1e-3 6.1e-3 7e-3]), ...
                                                'BiasInstability',deg2rad([7.5 8.1 4.9])*(1/3600), ...
                                                'AccelerationBias',deg2rad([0.572e-3 1.02e-3 0.408e-3]));
            
            LIS3MDL_params = magparams('MeasurementRange',G2microT(16), ...
                                       'ConstantBias',[0 0 0]);
            
            IMU = imuSensor('IMUType','accel-gyro-mag', ...
                            'SampleRate',100, ...
                            'MagneticField',[27.5550 -2.4169 -16.0849], ... % at lat long alt 0
                            'Accelerometer',test_accel_params, ...
                            'Gyroscope',test_gyro_params, ...
                            'Magnetometer',test_mag_params, ...
                            'RandomStream','Global stream', ...
                            'Seed',67);
            
            GPS = gpsSensor('SampleRate',100, ...
                            'ReferenceLocation',refLLA, ... % LLA of Georgia Tech
                            'PositionInputFormat','Local', ...
                            'HorizontalPositionAccuracy',0.1, ...
                            'VerticalPositionAccuracy',0.1);

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

        function [x2f, v2f, a2f] = state2imu(wbfb, x1f, v1f, a1f, qbf, r21b, v21b)
   
           qfb = quatconj(qbf'); % outputs 1x4
           r12b = -r21b;
           v12b = -v21b;
        
           % Assumptions
           a21b = [0;0;0];
           wfib = [0;0;0];
           alphabfb = [0;0;0];
           
           x2f = x1f - quat_rotate(qfb, r21b')';
           v2f = v1f - quat_rotate(qfb, (v12b + cross(wbfb,r12b))')';
           a2f = a1f - quat_rotate(qfb, (cross(wbfb,v21b) + cross(wbfb,cross(wbfb,r12b)) )')';
        
        end

        function microT_value = G2microT(gauss_value)

            microT_value = gauss_value * 100;
            
        end
        
    end

end