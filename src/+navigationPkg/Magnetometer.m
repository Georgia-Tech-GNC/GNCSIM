classdef Magnetometer < navigationPkg.Sensor&handle
    properties
        name = "Magnetometer"
        outputDimension = 3;
        noiseCovariance
        outputRate
        outputProbability
        observationMatrix
        date
        softIronMatrix
        hardIronBias
    end
    
    properties (Constant)
        SENSOR_TYPE = navigationPkg.SensorType.MAGNETOMETER
        idx = StateIndices
    end
    
    properties
        refLLA
        posI
    end
    
    methods
        function obj = Magnetometer(magParams)
            obj@navigationPkg.Sensor();
            obj.noiseCovariance = magParams.noiseCovariance;
            obj.outputRate = magParams.outputRate;
            obj.outputProbability = magParams.outputProbability;
            obj.observationMatrix = magParams.observationMatrix;
            obj.outputPeriod = 1 / obj.outputRate;
            obj.lastOutputTime = 0;
            obj.nextOutputTime = obj.outputPeriod;
            obj.refLLA = magParams.refLLA;
            obj.date = magParams.date; % Give in dd-mmm-yyyy 
            
            if isfield(magParams, 'softIronMatrix')
                obj.softIronMatrix = magParams.softIronMatrix;
            else
                obj.softIronMatrix = eye(3);
            end
            
            if isfield(magParams, 'hardIronBias')
                obj.hardIronBias = magParams.hardIronBias;
            else
                obj.hardIronBias = zeros(3, 1);
            end
        end
        
        function magReadingsBody = getMeasurement(obj, state, stateDot, currentTime)
            % Gives output in nanotesla in body frame
            
            % Get current position in ENU
            obj.posI = state(obj.idx.pos);
            
            % Convert ENU position to LLA
            [lat, lon, alt] = obj.ENU2LLA();
            
            % Get Magnetic Field in NED
            [magN, magE, magD] = wrldmagm(alt, lat, lon, decyear(obj.date, 'dd-mmm-yyyy'));
            
            % Convert from NED to ENU
            magFieldENU = [magE; magN; -magD];
            
            % Get body to ENU DCM
            body2EnuQuat = state(obj.idx.quat);
            body2EnuDCM = Utils.dirCosine(body2EnuQuat);
            
            % Rotate ENU magnetic field vector to body frame
            magVecBody = body2EnuDCM' * magFieldENU;
            
            % Get the magnetometer reading
            magReadingsBody = obj.softIronMatrix \ magVecBody + obj.hardIronBias;
        end
        
        function [lat, lon, alt] = ENU2LLA(obj)
            % Reference ECEF
            [xECEFref, yECEFref, zECEFref] = obj.LLA2ECEF(obj.refLLA(1), obj.refLLA(2), obj.refLLA(3));
            refECEF = [xECEFref; yECEFref; zECEFref];
            
            % Reference latitude and longitude (radians)
            phi = deg2rad(obj.refLLA(1));
            lambda = deg2rad(obj.refLLA(2));
            
            xENU = obj.posI(1);
            yENU = obj.posI(2);
            zENU = obj.posI(3);
            
            dx = -sin(lambda)*xENU - sin(phi)*cos(lambda)*yENU + cos(phi)*cos(lambda)*zENU;
            dy = cos(lambda)*xENU - sin(phi)*sin(lambda)*yENU + cos(phi)*sin(lambda)*zENU;
            dz = cos(phi)*yENU + sin(phi)*zENU;
            
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
            
            % Iterative latitude calculation (Bowring's formula)
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
        
        function [xECEF, yECEF, zECEF] = LLA2ECEF(obj, lat, lon, alt)
            % Converts from WGS84 to ECEF
            
            WGS84_E = 0.08181;
            WGS84_A = 6378137.0;
            
            % Convert latitude and longitude from degrees to radians
            phi = deg2rad(lat);
            lambda = deg2rad(lon);
            
            % Convert LLA to ECEF
            N = WGS84_A / sqrt(1 - (WGS84_E^2 * (sin(phi))^2));
            
            xECEF = (N + alt) * cos(phi) * cos(lambda);
            yECEF = (N + alt) * cos(phi) * sin(lambda);
            zECEF = (N * (1.0 - WGS84_E * WGS84_E) + alt) * sin(phi);
        end
    end
end