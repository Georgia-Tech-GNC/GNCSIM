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
                error('Dry intertia values must be positive doubles.')
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
        % End Setter Functions

        % Start Getter Functions
        function wetMass = get_wetMass(obj)
            wetMass = obj.wetMass;
        end
            
        function dryMass = get_dryMass(obj)
            dryMass = obj.wetMass;
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

        function [FA_b,MA_b,l_p_curr,n_a_curr,n_r_curr] = calcAeroBody3(obj,alpha,vel,rho,CG,angVel)
            %calcAeroBody3 Wrapper for aero model for calculating aero forces and 
            % moments for the nonlinear rocket model. Aero equations are only valid 
            % up to roughly 10 degrees angle of attack.
            %
            % alpha - angle of attack (rad)
            % vel - velocity (m/s)
            % rho - air density (kg/m^3)
            % CG - location of CG (m)
            %
            % FA_b - aero force in body frame (N)
            % MA_b - aero moment in body frame about current CG (Nm)

            %% Convert from metric to english for aero fittings
            
            alphaRad = alpha;
            alphaDeg = alphaRad * (180/pi) ; % rad to deg
            vel = vel * 3.281; % m/s to ft/s
            rho = rho * 0.00194;% kg/m^3 to slug/ft^3
            CG = CG * 3.281; % m to ft
            % angVel = rad2deg(angVel); % rad to deg
        
            % The following code uses Rushil's aero inputs and nomenclature
            % T = thrust (lb)
            % a = angle of attack (deg)
            % ad = time derivative of a
            % add = second derivative of a
            % rho = density (slug/ft^3)
            % v = velocity (ft/s)
            % CG = CG location from nosetip (ft)
            % h = altitude (ft)
            % _a = due to angle of attack
            % _r = due to yaw
            % _p = due to roll
            % l, m, n - moments in x,y,z directions
            % p, q, r - angular rates in x,y,z directions (rad/s)
        
            %% Aero fittings (in British units)
            CM = @(a, v) a .* ((4.12421 ./ (27.81553 + 16.83430 .* sqrt(1 - (v./1097.40).^2))) + -0.00014 .* a + 0.01273);
            CD = @(a) 0.00253 * a.^2 + 0.13932;
            CL = @(a, v) 0.64181 .* a .* (0.64181 ./ (3.59306 + sqrt(1 - (v./1097.40).^2)));
            
            Mo = @(a, v, rho) CM(a, v) .* 0.5 .* rho .* v.^2 .* 0.9375;
            D_a = @(a, v, rho) CD(a) .* 0.5 .* rho .* v.^2 .* 0.9375;
            L_a = @(a, v, rho) CL(a,v) .* 0.5 .* rho .* v.^2 .* 0.9375;
            %L_r = @(rho,v,r,CG) 0.5*rho*v^2*0.9375 * (2.61799)/(sqrt(1+0.797*(1-(v*1116.45)^2))) * asin(r*(10.0948-CG)/v);
            L_r = @(rho, v, r, CG) 0.5 * rho * v^2 * 0.9375 * (2.61799 / (1 + sqrt(1 + 0.797628 * (1 - (v / 1116.45)^2)))) * asin(r * (10.3264 - CG) / v);

            %% Aero force in body frame
            Lmag = L_a(alphaDeg,vel,rho) + L_r(rho,vel,angVel(1),CG);
            L_b = Lmag .* [sin(alphaRad);-cos(alphaRad);0];
            D_b = D_a(alphaDeg,vel,rho) .* [-cos(alphaRad);-sin(alphaRad);0];
            FA_b = L_b + D_b; % lbf
            FA_b = FA_b .* (1/0.225); % lbf to N
        
            %% Aero moment in body frame about current CG
        
            n_a = @(a, v, rho) CM(a, v) * 0.5 * rho * v.^2 * 0.9375; % corrective moment due to a (angle of attack)
            %n_r = @(rho,v,r,CG) 0.5*rho*v^2*0.9375 * (2.61799)/(sqrt(1+0.797*(1-(v*1116.45)^2))) * asin(r*(10.3264-CG)/v) * (10.3264-CG);
            n_r = @(rho, v, r, CG) 0.5 * rho * v^2 * 0.9375 * (2.61799 / (1 + sqrt(1 + 0.797628 * (1 - (v / 1116.45)^2)))) * asin(r * (10.3264 - CG) / v) * (10.3264 - CG);

            l_p = @(rho,v,p) (((4.*((2.*pi)./sqrt(1-(v./1116.4).^2)).*p)./(0.4688.*v)).*0.1897).*0.5.*rho.*v.^2.*0.4688;
        
            % Need to perform moment transfer
            % Aero fittings for moment are taken about an OpenRocket CG0
            % Need to perform a moment transfer to SolidWorks CG0 then to current
            % CG0
        
            % Difference btw OpenRocket CG0 and SolidWorks CG0
            % TODO assuming 0 for now will need to change later
            CG0diff = 0; % ft
        
            % Moment about OpenRocket CG0
            % MA_b_CG0 = [0;0;Mo(alphaDeg,vel,rho)];
        
            if vel == 0
                MA_b_CG0_x = 0;
            else
                l_p_curr = l_p(rho,vel,angVel(1));
                MA_b_CG0_x = l_p_curr;
            end
            MA_b_CG0_y = 0;
            n_a_curr = n_a(alphaDeg,vel,rho);
            n_r_curr = n_r(rho,vel,angVel(3),CG);
            MA_b_CG0_z = n_a_curr + n_r_curr;
        
            MA_b_CG0 = [MA_b_CG0_x;
                        MA_b_CG0_y;
                        MA_b_CG0_z];
            
            % Moment about SolidWorks CG0
            % r points from CG0_corr to CG0
            %     MA_b_CG0_corr = MA_b_CG0 + cross(FA_b,-1.*[(CG+CG0diff);0;0]);
        
            % Moment about current CG (WE ARE NOT USING THIS ANYMORE)
            MA_b_temp = MA_b_CG0 + cross(FA_b*0.225,-1*[CG;0;0]);
            MA_b_temp = MA_b_temp .* 1.36;
        
            MA_b = MA_b_CG0 .* 1.36; % lbf-ft to Nm
        
            % get error from not assuming moment transfer
            temp = MA_b_temp - MA_b;
        
        
        
        
        
        
        end

    end
end
