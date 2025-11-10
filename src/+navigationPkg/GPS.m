classdef GPS < navigationPkg.Sensor&handle
    % Model of in-flight GPS

    properties 
        name = "GPS"
        outputDimension = 3;
        noiseCovariance
        outputRate           % Hz
        outputProbability         % 
        observationMatrix 
    end

    properties (Constant)
        SENSOR_TYPE = navigationPkg.SensorType.GPS
    end

    properties
        refLLA
        posI
        gpsBuiltIn    
    end


    methods

        function obj = GPS(gpsParams)

            obj@navigationPkg.Sensor()
            obj.noiseCovariance = gpsParams.noiseCovariance;
            obj.outputRate = gpsParams.outputRate;
            obj.outputProbability = gpsParams.outputProbability;
            obj.observationMatrix = gpsParams.observationMatrix;

            obj.outputPeriod = 1 / obj.outputRate;
            obj.lastOutputTime = 0;
            obj.nextOutputTime = obj.outputPeriod;

            obj.refLLA = gpsParams.refLLA;
            obj.gpsBuiltIn = gpsSensor('SampleRate', obj.outputRate, ...
                                'ReferenceLocation', obj.refLLA, ...
                                'PositionInputFormat', 'Geodetic', ...
                                'VerticalPositionAccuracy', gpsParams.verticalAccuracy, ...
                                'HorizontalPositionAccuracy', gpsParams.horizontalAccuracy, ...
                                'VelocityAccuracy', gpsParams.velocityAccuracy);

        end

        function measurement = getMeasurement(obj, state, stateDot, currentTime)
            if obj.isOutputReady(currentTime)
                obj.posI = state(2:4);
                [lat, lon, alt] = obj.ENU2LLA();
                llaPos = [lat, lon, alt];
                [latM, lonM, altM] = obj.gpsBuiltIn(llaPos,state(5:7)');
                measurement = [latM, lonM, altM];
                obj.scheduleNextOutput(currentTime);
            else
                measurement = [];
            end

        end

        function [lat, lon, alt] = ENU2LLA(obj)

            % Converts from NED coordinates back to LLA (WGS84)

            % Reference ECEF
            refECEF = lla2ecef([obj.refLLA(1), obj.refLLA(2), obj.refLLA(3)]);

            % Reference latitude and longitude (radians)
            phi = deg2rad(obj.refLLA(1));
            lambda = deg2rad(obj.refLLA(2));

            xENU = obj.posI(1);
            yENU = obj.posI(2);
            zENU = obj.posI(3);

            dx = -sin(lambda)*xENU - sin(phi)*cos(lambda)*yENU + cos(phi)*cos(lambda)*zENU;
            dy = cos(lambda)*xENU - sin(phi)*sin(lambda)*yENU + cos(phi)*sin(lambda)*zENU;
            dz =  cos(phi)*yENU + sin(phi)*zENU;
            
            xECEF = refECEF(1) + dx;
            yECEF = refECEF(2) + dy;
            zECEF = refECEF(3) + dz;

            % Convert ECEF → LLA
            [lat, lon, alt] = obj.ECEF2LLA(xECEF, yECEF, zECEF);
            

        end

        function [lat, lon, alt] = ECEF2LLA(obj, xECEF, yECEF, zECEF)
            % Converts from ECEF to WGS84 Latitude, Longitude, Altitude

            WGS84_A = 6378137.0;         
            WGS84_E = 0.08181;           
            WGS84_B = sqrt(WGS84_A^2 * (1 - WGS84_E^2)); 
            ep = sqrt((WGS84_A^2 - WGS84_B^2)/WGS84_B^2);

            % Longitude
            lon = atan2(yECEF, xECEF);

            % Iterative latitude calculation (Bowring’s formula)
            p = sqrt(xECEF^2 + yECEF^2);
            th = atan2(WGS84_A * zECEF, WGS84_B * p);
            lat = atan2(zECEF + ep^2 * WGS84_B * sin(th)^3, ...
                        p - WGS84_E^2 * WGS84_A * cos(th)^3);

            % Radius of curvature
            N = WGS84_A / sqrt(1 - WGS84_E^2 * sin(lat)^2);

            % Altitude
            alt = p / cos(lat) - N;

            % Convert to degrees
            lat = rad2deg(lat);
            lon = rad2deg(lon);

        end
    end
end