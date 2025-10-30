classdef GPS < Sensor
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
        SENSOR_TYPE = SensorType.GPS
    end

    properties
        refLLA
        posI
        gpsBuiltIn    
    end


    methods

        function obj = GPS(gps_params)
        
            obj@Sensor()
            obj.noiseCovariance = gps_params.noiseCovariance;
            obj.outputRate = gps_params.outputRate;
            obj.outputProbability = gps_params.outputProbability;
            obj.observationMatrix = gps_params.observationMatrix;

            obj.outputPeriod = 1 / obj.outputRate;
            obj.lastOutputTime = 0;
            obj.nextOutputTime = obj.outputPeriod;

            obj.refLLA = gps_params.refLLA;
            obj.gpsBuiltIn = gpsSensor('SampleRate', obj.outputRate, ...
                                'ReferenceLocation', obj.refLLA, ...
                                'PositionInputFormat', 'Geodetic', ...
                                'VerticalPositionAccuracy', gps_params.verticalAccuracy, ...
                                'HorizontalPositionAccuracy', gps_params.horizontalAccuracy, ...
                                'VelocityAccuracy', gps_params.velocityAccuracy);
        end

        function measurement = getMeasurement(obj, state, stateDot, currentTime)
            
            if obj.isOutputReady(currentTime)
                obj.posI = state(2:4);
                llaPos = NED2lla();
                measurement = obj.gpsBuiltIn(llaPos,state(5:7));
            else
                measurement = [];
            end

            obj.scheduleNextOutput(currentTime);

        end

        function [lat, lon, alt] = NED2LLA(obj)

            % Converts from NED coordinates back to LLA (WGS84)

            % Reference ECEF
            refECEF = LLA2ECEF(obj.refLLA(1), obj.refLLA(2), obj.refLLA(3));

            % Reference latitude and longitude (radians)
            phi = deg2rad(obj.refLLA(1));
            lambda = deg2rad(obj.refLLA(2));

            xNED = obj.inertialPos(1);
            yNED = obj.inertialPos(2);
            zNED = obj.inertialPos(3);

            dx = -sin(phi)*cos(lambda)*xNED - sin(lambda)*yNED - cos(phi)*cos(lambda)*zNED;
            dy = -sin(phi)*sin(lambda)*xNED + cos(lambda)*yNED - cos(phi)*sin(lambda)*zNED;
            dz =  cos(phi)*xNED - sin(phi)*zNED;
            
            xECEF = refECEF(1) + dx;
            yECEF = refECEF(2) + dy;
            zECEF = refECEF(3) + dz;

            % Convert ECEF → LLA
            [lat, lon, alt] = ECEF2LLA(xECEF, yECEF, zECEF);

        end

        function [lat, lon, alt] = ECEF2LLA(xECEF, yECEF, zECEF)
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