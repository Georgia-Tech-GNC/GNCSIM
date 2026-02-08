% Main Authors: Aditya Swami (aswami8@gatech.edu)
%               Karsten Caillet (kcaillet3@gatech.edu)

classdef ParachuteDynamics < handle
    properties
        % Parachute physical properties
        canopyMass              % Canopy mass (kg)
        canopyDiameter          % Canopy diameter (m)
        referenceArea           % Parachute reference area (m^2)
        dragCoefficient = 0.75  % Parachute drag coefficient (dimensionless)
        deployAltitude          % Altitude the chute is deployed at (m)
        
        % Environment constants
        airDensity              % Air density at sea level (kg/m^3)
        gravity                 % Gravitational acceleration (m/s^2)
    end
    
    methods
        function obj = ParachuteDynamics(canopyDiameter, canopyMass, dragCoeff, deployAlt)
            % Constructor for simplified parachute dynamics model
            %
            % Inputs:
            %   canopyDiameter - Canopy diameter (m)
            %   canopyMass     - Canopy mass (kg)
            %   dragCoeff      - Drag coefficient (dimensionless)
            %   deployAlt      - Deployment altitude (m AGL)
            
            % Set parachute properties
            obj.canopyDiameter = canopyDiameter;
            obj.canopyMass = canopyMass;
            obj.dragCoefficient = dragCoeff;
            obj.deployAltitude = deployAlt;
            
            % Set environmental constants
            obj.airDensity = 1.225;  % kg/m^3 (sea level, ISA)
            obj.gravity = 9.81;      % m/s^2
            
            % Calculate reference area (projected area of circular canopy)
            obj.referenceArea = pi * (canopyDiameter/2)^2;
        end
        
        function forceAeroBody = getForcesAndMoments(obj, ...
                velocityRocketBody, quaternionRocket, disturbances)
            % Compute aerodynamic forces and moments from parachute
            %
            % Inputs:
            %   velocityRocketBody - Rocket velocity in body frame (m/s)
            %   quaternionRocket   - Quaternion: inertial to body frame [q0; q1; q2; q3]
            %   disturbances       - Struct with .wind field (inertial frame, m/s)
            %
            % Outputs:
            %   forceAeroBody  - Total aerodynamic + weight force in body frame (N)
            
            % Direction cosine matrices for frame transformations
            dcmInertialToBody = Utils.dirCosine(quaternionRocket);
            dcmBodyToInertial = dcmInertialToBody';
            
            % Transform rocket velocity to inertial frame
            velocityRocketInertial = dcmBodyToInertial * velocityRocketBody;
            
            % Compute airspeed (velocity relative to air mass)
            airspeedInertial = velocityRocketInertial - disturbances.wind;
            airspeedMagnitude = norm(airspeedInertial);
            
            % Compute dynamic pressure
            dynamicPressure = 0.5 * obj.airDensity * airspeedMagnitude^2;
            
            % Compute drag force (opposes relative motion through air)
            if airspeedMagnitude > 1e-6
                forceAeroInertial = -dynamicPressure * obj.referenceArea * ...
                                    obj.dragCoefficient * (airspeedInertial / airspeedMagnitude);
            else
                forceAeroInertial = [0; 0; 0];
            end
            
            % Add parachute weight (acts downward in inertial frame)
            forceGravityInertial = [0; 0; -obj.canopyMass * obj.gravity];
            forceTotalInertial = forceAeroInertial + forceGravityInertial;
            
            % Transform total force to body frame
            forceAeroBody = dcmInertialToBody * forceTotalInertial;
        end
    end
end