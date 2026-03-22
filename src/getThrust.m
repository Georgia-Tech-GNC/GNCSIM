function currentThrust = getThrust(t, thrustCurve, burnTime)
            [m,~] = size(thrustCurve);
            % Errors for incompatible inputs
            if ~isnumeric(t)
                error('Error: Time must be a double')
            end
            % initialization for while loop
            arr = thrustCurve;
            lowerBoundTime = [];
            upperBoundTime = [];
            lowerBoundThrust = [];
            upperBoundThrust = [];
            skipInterpolation = false;
            found = false;
            i = 1;
            if t > burnTime
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