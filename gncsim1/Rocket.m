classdef Rocket < handle
    %Summary of this class goes here
    %   Detailed explanation goes here

    properties (SetAccess = private)
        wetMass; % kg
        dryMass; % kg
        wetMOI;%3x3, (kgm^2)
        dryMOI; %3x3, (kgm^2)
        initialCG; %distance from nosecone to CG with loaded motor (m)
        finalCG; %distance from nosecone to CG with empty motor (m)
        CP; % distance from nosecone to CP, doesn't change (m)
        length; % nosecone to bottom of motor (m)
        diameter; % tube diameter (m)
        radius; % tube diameter/2 (m)

        burnTime; % (s)
        thrustCurve; % time history of thrust (s, N)

        mainChuteCd; % Cd of the Main Parachute
        mainChuteArea; % (m^2) of the Main Parachute
        drogueChuteCd;  % Cd of the Drogue Parachute
        drogueChuteArea; % (m^2) of the Drogue Parachute
    end

    methods
        function obj = Rocket()
            %Construct an instance of this class
            %   Detailed explanation goes here
        end
        
        % Start Setter Functions
        function obj= set_wetMass(obj, wet_mass)
            if (~isnumeric(wet_mass) || wet_mass < 0)
                error('Wet mass value must be a positive double.')
            else
                obj.wetMass = wet_mass;
            end  
        end

        function obj= set_dryMass(obj, dry_mass)
            if (~isnumeric(dry_mass) || dry_mass < 0)
                error('Dry mass value must be a positive double.')
            else
                obj.dryMass = dry_mass;
            end  
        end

        function obj= set_dryMOI(obj, dry_inertias)
            [r,c] = size(dry_inertias);
            if ~(isnumeric(dry_inertias) && all(diag(dry_inertias) > 0) && (r == c))
                error('Dry inertia values must be positive doubles.')
            else
                obj.dryMOI = dry_inertias;
            end  
        end

        function obj= set_wetMOI(obj, wet_inertias)
            [r,c] = size(wet_inertias);
            if ~(isnumeric(wet_inertias) && all(diag(wet_inertias) > 0) && (r == c))
                error('Wet inertia values must be positive doubles, and the inertia matrix must be square.')
            else
                obj.wetMOI = wet_inertias;
            end  
        end

        function obj= set_initialCG(obj, initial_CG)
            if (~isnumeric(initial_CG) || initial_CG < 0)
                error('CG value must be a positive double.')
            else
                obj.initialCG = initial_CG;
            end
        end

        function obj= set_finalCG(obj, final_CG)
            if (~isnumeric(final_CG) || final_CG < 0)
                error('CG value must be a positive double.')
            else
                obj.finalCG = final_CG;
            end
        end

        function obj= set_CP(obj, CP)
            if (~isnumeric(CP) || final_CP < 0)
                error('CP value must be a positive double.')
            else
                obj.CP = CP;
            end
        end

        function obj= set_length(obj, length)
            if (~isnumeric(length) || length < 0)
                error('Rocket length value must be a positive double.')
            else
                obj.length = length;
            end
        end

        function obj= set_diameter(obj, diameter)
            if (~isnumeric(diameter) || diameter < 0)
                error('Rocket diameter value must be a positive double.')
            else
                obj.diameter = diameter;
            end
        end

        function obj= set_radius(obj, radius)
            if (~isnumeric(radius) || radius < 0)
                error('Rocket radius value must be a positive double.')
            else
                obj.radius = radius;
            end
        end

        function obj= set_burnTime(obj, burn_time_value)
            if (~isnumeric(burn_time_value) || burn_time_value < 0)
                error('Burn time value must be a positive double.')
            else
                obj.burnTime = burn_time_value;
            end
        end

        function obj = set_mainChuteCd(obj,newCdValue)
            obj.mainChuteCd = newCdValue;
        end

        function obj = set_drogueChuteCd(obj,newCdValue)
            obj.drogueChuteCd = newCdValue;
        end

        function obj = set_mainChuteArea(obj,newAreaValue)
            obj.mainChuteArea = newAreaValue;
        end

        function obj = set_drogueChuteArea(obj,newAreaValue)
            obj.drogueChuteArea = newAreaValue;
        end
        % End Setter Functions

        % Start Getter Functions
        function wetMass = get_wetMass(obj)
            wetMass = obj.wetMass;
        end
            
        function dryMass = get_dryMass(obj)
            dryMass = obj.dryMass;
        end

        function dryInertias = get_dryMOI(obj)
            dryInertias = obj.dryMOI;
        end

        function wetInertias = get_wetMOI(obj)
            wetInertias = obj.wetMOI;
        end

        function initialCG = get_initialCG(obj)
            initialCG = obj.initialCG;
        end

        function finalCG = get_finalCG(obj)
            finalCG = obj.finalCG;
        end

        function CP = get_CP(obj)
            CP = obj.CP;
        end

        function length = get_length(obj)
            length = obj.length;
        end

        function diameter = get_diameter(obj)
            diameter = obj.diameter;
        end

        function radius = get_radius(obj)
            radius = obj.radius;
        end

        function burnTime = get_burnTime(obj)
            burnTime = obj.burnTime;
        end

        function thrustCurve = get_thrustCurve(obj)
            thrustCurve = obj.thrustCurve;
        end

        function mainChuteCdValue = get_mainChuteCd(obj)
            mainChuteCdValue = obj.mainChuteCd;
        end

        function mainChuteAreaValue = get_mainChuteArea(obj)
            mainChuteAreaValue = obj.mainChuteArea;
        end

        function drogueChuteCdValue = get_drogueChuteCd(obj)
            drogueChuteCdValue = obj.drogueChuteCd;
        end

        function drogueChuteAreaValue = get_drogueChuteArea(obj)
            drogueChuteAreaValue = obj.drogueChuteArea;
        end
        % End Getter Functions
        
        function obj= set_thrustCurve(obj, thrust_arr)
            [~,c] = size(thrust_arr);
            if c ~= 2 || (thrust_arr(1, 1) == 0 && thrust_arr(1,2) == 0)
                error(['Error in dimensions of thrust array values. ' ...
                    'Time must be first column, and thrust must be second column.'])
            else
                obj.thrustCurve = thrust_arr;
            end
        end   

        function currentThrust = getThrust(obj, t)
            [m,~] = size(obj.thrustCurve);
            % Errors for incompatible inputs
            if ~isnumeric(t)
                error('Error: Time must be a double')
            end
            % initialization for while loop
            arr = obj.thrustCurve;
            lowerBoundTime = [];
            upperBoundTime = [];
            lowerBoundThrust = [];
            upperBoundThrust = [];
            skipInterpolation = false;
            found = false;
            i = 1;
            if t > obj.burnTime
                currentThrust = 0;
                return
            end
            % while loop that outputs values to be linearily interpolated
            while i <= m & ~found
                if arr(i, 1) < t && arr(i + 1, 1) > t
                    lowerBoundTime = arr(i, 1);
                    upperBoundTime = arr(i, 2);
                    lowerBoundThrust = arr(i, 2);
                    upperBoundThrust = arr(i + 1, 2);
                    found = true;
                elseif arr(i, 1) == t
                    skipInterpolation = true;
                    currentThrust = arr(i, 2);
                    found = true;
                else
                    i = i + 1;
                end
            end
            % linear interpolation for thrust
            if ~skipInterpolation
                deltaThrust = upperBoundThrust - lowerBoundThrust;
                deltat = upperBoundTime - lowerBoundTime;
                m = deltaThrust / deltat;
                b = upperBoundThrust - (m * upperBoundTime);
                currentThrust = (m * t) + b;
            end
        end
    end
end
