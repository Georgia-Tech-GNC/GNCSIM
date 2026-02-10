% Main Authors: Aditya Swami (aswami8@gatech.edu)
%               Karsten Caillet (kcaillet3@gatech.edu)

classdef ParachuteDynamics < handle
    properties
        name (1,1) string                    % Name of the parachute
        isDeployed (1,1) logical = false      % Track if parachute has been deployed
        % Parachute physical properties
        canopyMass (1,1) double              % Canopy mass (kg)
        canopyDiameter (1,1) double          % Canopy diameter (m)
        referenceArea (1,1) double           % Parachute reference area (m^2)
        dragCoefficient (1,1) double         % Parachute drag coefficient (dimensionless)
        deployAltitude (1,1) double          % Altitude the chute is deployed at (m)
        
        % Environment constants
        airDensity (1,1) double              % Air density at sea level (kg/m^3)
        gravity (1,1) double                 % Gravitational acceleration (m/s^2)

        % Linked list structure
        nextParachute rocketPkg.ParachuteDynamics {mustBeScalarOrEmpty} = rocketPkg.ParachuteDynamics.empty % Reference to next parachute in sequence
    end
    
    methods
        function obj = ParachuteDynamics(name, canopyDiameter, canopyMass, dragCoeff, deployAlt)
            % Constructor for simplified parachute dynamics model
            %
            % Inputs:
            %   name           - Name of the parachute
            %   canopyDiameter - Canopy diameter (m)
            %   canopyMass     - Canopy mass (kg)
            %   dragCoeff      - Drag coefficient (dimensionless)
            %   deployAlt      - Deployment altitude (m AGL)
            
            % Set parachute properties
            obj.name = name;
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
            arguments
                obj
                velocityRocketBody (3,1) double
                quaternionRocket (4,1) double
                disturbances struct
            end
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
            cBI = Utils.dirCosine(quaternionRocket);
            cIB = cBI';
            
            % Transform rocket velocity to inertial frame
            velocityRocketInertial = cIB * velocityRocketBody;
            
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
            forceAeroBody = cBI * forceTotalInertial;
        end

        function shouldTransition = shouldDeployNext(obj, altitude)
            arguments
                obj
                altitude (1,1) double
            end
            % Check if conditions are met to transition to next parachute
            if isempty(obj.nextParachute)
                shouldTransition = false;
            else
                shouldTransition = altitude <= obj.nextParachute.deployAltitude;
            end
        end

        function next = getNext(obj)
            % Get the next parachute in sequence
            next = obj.nextParachute;
        end

        function setNext(obj, nextChute)
            arguments
                obj
                nextChute rocketPkg.ParachuteDynamics
            end
            % Set the next parachute in sequence
            obj.nextParachute = nextChute;
        end
    end
end