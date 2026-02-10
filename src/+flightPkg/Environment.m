% Authors: Hridai Ambati (hambati3@gatech.edu), Zhanna Shorokhova
% (zshorokhova3@gatech.edu)
% 
% (ported from GNCSIM 1.0, modified to implement 3D wind,
%  updated with smoother turbulence, veer limiting, and RNG control)

classdef Environment < handle
    %ENVIRONMENT Atmospheric wind model with shear, veer, and turbulence.
    %   Builds a 3D wind field as a function of altitude using:
    %     - Power-law vertical shear
    %     - Logarithmic veer (direction change with altitude)
    %     - Turbulence scaled with mean wind and vertically correlated
    
    properties

        %--------------------------------------------------------------
        % Altitude grid
        %--------------------------------------------------------------
        alts (1,:) double                   % altitude vector (m)
        stepSize (1,1) double = 1           % vertical resolution (m)
        maxAltitude (1,1) double = 10000    % maximum altitude (m)

        %--------------------------------------------------------------
        % Shear law parameters (Power Law)
        % U(z) = U0 * (z / zRef)^alpha
        %--------------------------------------------------------------
        U0 (1,1) double = 5                 % reference wind speed at zRef (m/s)
        zRef (1,1) double = 10              % reference height (m)
        alpha (1,1) double = 0.14           % terrain exponent (0.1–0.3)

        windMag (1,:) double                % mean wind magnitude profile U(z)

        %--------------------------------------------------------------
        % Turbulence intensity scaling
        % sigma(z) = I0 * U(z)
        %--------------------------------------------------------------
        I0 (1,1) double = 0.10              % turbulence intensity (5–15%)
        sigmaVec (1,:) double               % turbulence amplitude at each altitude

        %--------------------------------------------------------------
        % Veer (Coriolis rotation)
        % theta(z) = theta0 + k * log(z/zRef + 1)
        %--------------------------------------------------------------
        theta0 (1,1) double = 0             % base wind direction (radians)
        k (1,1) double = 0.1                % veer coefficient
        thetaVec (1,:) double               % direction vs altitude (radians)

        %--------------------------------------------------------------
        % Turbulence structure parameters
        %--------------------------------------------------------------
        rhoTurb (1,1) double = 0.95         % vertical AR(1) correlation coefficient
        wScale (1,1) double = 0.5           % vertical turbulence scale vs horizontal

        %--------------------------------------------------------------
        % RNG control (optional)
        %--------------------------------------------------------------
        rngSeed (:,:) double = []           % if set, used to seed RNG for reproducibility

        %--------------------------------------------------------------
        % Final 3D wind field
        % windVec3D(n,:) = [Wx(z), Wy(z), Wz(z)]
        %--------------------------------------------------------------
        windVec3D (:,3) double              % Nx3 matrix of 3D wind vectors

    end

    methods

        %--------------------------------------------------------------
        % Constructor
        % Builds the full 3D wind field upon object creation.
        %
        % Optional INPUT:
        %   rngSeed - scalar used to seed MATLAB's RNG (for reproducibility)
        %
        % Pipeline:
        %   1. Build altitude grid
        %   2. Compute shear-law mean wind magnitude U(z)
        %   3. Compute turbulence amplitude σ(z)
        %   4. Compute veer (direction vs altitude)
        %   5. Compute final 3D wind vectors [Wx, Wy, Wz]
        %--------------------------------------------------------------
        function obj = Environment(rngSeed)
            if nargin > 0
                obj.rngSeed = rngSeed;
                rng(obj.rngSeed);
            end

            obj.buildAltitudeGrid();
            obj.computeShearLaw();
            obj.computeTurbulence();
            obj.computeVeer();
            obj.computeWind3D();
        end


        %--------------------------------------------------------------
        % getWindVector
        % Returns the 3D wind vector at a given altitude.
        %
        % NOTE ON LOOKUP METHOD:
        % This uses nearest‑neighbor indexing (snapping) to return the wind vector.
        % This is appropriate for the current 1 m altitude resolution.
        %
        % If stepSize is changed to a coarser value (e.g., >1 m), or if smoother
        % altitude variation is required for GN&C, replace this with linear
        % interpolation:
        %
        %     windVec = interp1(obj.alts, obj.windVec3D, alt, 'linear', 'extrap');
        %
        % Snapping is fast and adequate for fine grids; interpolation is recommended
        % for coarse grids or high‑fidelity simulations.
        %
        % INPUT:
        %   alt  - altitude (m)
        %
        % OUTPUT:
        %   windVec = [Wx, Wy, Wz] at the nearest altitude index
        %--------------------------------------------------------------
        function windVec = getWindVector(obj, alt)
            arguments
                obj
                alt (1,1) double
            end
            % Map altitude to nearest index (alts start near 0 with stepSize)
            idx = round(alt / obj.stepSize) + 1;
            idx = max(min(idx, numel(obj.alts)), 1);
            windVec = obj.windVec3D(idx, :);
        end


        %--------------------------------------------------------------
        % buildAltitudeGrid
        % Creates the altitude vector from a small epsilon to maxAltitude
        % using stepSize.
        %
        % OUTPUT:
        %   obj.alts = [eps, eps+stepSize, ..., maxAltitude]
        %
        % Notes:
        %   - Starting at eps avoids log(0) and 0^alpha
        %   - This grid is the backbone for all wind profile computations.
        %--------------------------------------------------------------
        function buildAltitudeGrid(obj)
            epsAlt = 1e-3; % small positive altitude to avoid singularities
            obj.alts = epsAlt:obj.stepSize:obj.maxAltitude;
        end


        %--------------------------------------------------------------
        % computeShearLaw
        % Computes the mean wind magnitude profile using the power law:
        %
        %   U(z) = U0 * (z / zRef)^alpha
        %
        % OUTPUT:
        %   obj.windMag = vector of mean wind speeds at each altitude
        %
        % Notes:
        %   - Implements the vertical shear assumption
        %   - Ensures wind increases with altitude (for α > 0)
        %   - Applies a small floor to avoid near-zero speeds
        %--------------------------------------------------------------
        function computeShearLaw(obj)
            obj.windMag = obj.U0 * (obj.alts / obj.zRef).^obj.alpha;

            % Apply a small floor to avoid numerical issues at very low z
            minWind = 0.1; % m/s
            obj.windMag = max(obj.windMag, minWind);
        end


        %--------------------------------------------------------------
        % computeTurbulence
        % Computes turbulence amplitude at each altitude using:
        %
        %   σ(z) = I0 * U(z)
        %
        % OUTPUT:
        %   obj.sigmaVec = turbulence amplitude at each altitude
        %
        % Notes:
        %   - Turbulence grows with altitude because U(z) grows
        %--------------------------------------------------------------
        function computeTurbulence(obj)
            obj.sigmaVec = obj.I0 * obj.windMag;
        end


        %--------------------------------------------------------------
        % computeVeer
        % Computes wind direction vs altitude using a smooth veer law:
        %
        %   θ(z) = θ0 + k * log(z / zRef + 1)
        %
        % OUTPUT:
        %   obj.thetaVec = direction (radians) at each altitude
        %
        % Notes:
        %   - Models Coriolis-induced clockwise rotation with altitude
        %   - log(z/zRef + 1) avoids log(0)
        %   - Direction change is optionally limited to a reasonable range
        %--------------------------------------------------------------
        function computeVeer(obj)
            obj.thetaVec = obj.theta0 + obj.k * log(obj.alts / obj.zRef + 1);

            % Optional: limit total veer to ±45 degrees from base direction
            maxDelta = deg2rad(45);
            obj.thetaVec = max(min(obj.thetaVec, obj.theta0 + maxDelta), ...
                               obj.theta0 - maxDelta);
        end


        %--------------------------------------------------------------
        % computeWind3D
        % Computes the final 3D wind vector at each altitude:
        %
        %   Wx(z) = U(z)cos(θ(z)) + noise_x(z)
        %   Wy(z) = U(z)sin(θ(z)) + noise_y(z)
        %   Wz(z) = noise_z(z)
        %
        % Turbulence is modeled as vertically correlated AR(1) noise:
        %
        %   n_i = ρ n_{i-1} + sqrt(1-ρ^2) σ_i ξ_i,  ξ_i ~ N(0,1)
        %
        % OUTPUT:
        %   obj.windVec3D = Nx3 matrix of wind vectors
        %
        % Notes:
        %   - Horizontal components include mean + turbulence
        %   - Vertical component is turbulence only, scaled by wScale
        %   - Produces a smooth, continuous wind field in altitude
        %--------------------------------------------------------------
        function computeWind3D(obj)
            N = numel(obj.alts);
        
            rho = obj.rhoTurb;
            sigmaH = obj.sigmaVec(:);          % horizontal turbulence amplitude
            sigmaV = obj.wScale * sigmaH;      % vertical turbulence amplitude
        
            %--------------------------------------------------------------
            % Vectorized AR(1) turbulence using MATLAB's filter()
            %
            % AR(1):  n_i = rho * n_{i-1} + sqrt(1 - rho^2) * sigma(i) * xi_i
            %
            % filter(b, a, x) implements:
            %   y[n] = b * x[n] + a2 * y[n-1]
            %
            % Choose:
            %   a = [1, -rho]
            %   b = sqrt(1 - rho^2)
            %
            % Input x = sigma .* randn(N,1)
            %--------------------------------------------------------------
            a = [1, -rho];
            b = sqrt(1 - rho^2);
        
            noiseX = filter(b, a, sigmaH .* randn(N,1));
            noiseY = filter(b, a, sigmaH .* randn(N,1));
            noiseZ = filter(b, a, sigmaV .* randn(N,1));
        
            % Mean horizontal wind components
            Wx_mean = obj.windMag(:) .* cos(obj.thetaVec(:));
            Wy_mean = obj.windMag(:) .* sin(obj.thetaVec(:));
        
            % Add turbulence
            Wx = Wx_mean + noiseX;
            Wy = Wy_mean + noiseY;
            Wz = noiseZ;
        
            obj.windVec3D = [Wx, Wy, Wz];
        end

        %--------------------------------------------------------------
        % getAltitudes
        % Returns the altitude grid used for wind computations.
        %
        % OUTPUT:
        %   alts = altitude vector
        %--------------------------------------------------------------
        function alts = getAltitudes(obj)
            alts = obj.alts;
        end


        %--------------------------------------------------------------
        % setMaxAltitude
        % Updates the maximum altitude and recomputes the entire wind field.
        %
        % INPUT:
        %   altitude - new maximum altitude (m)
        %
        % Notes:
        %   - Rebuilds altitude grid
        %   - Recomputes shear, turbulence, veer, and 3D wind
        %   - If rngSeed is set, RNG is reseeded for reproducibility
        %--------------------------------------------------------------
        function obj = setMaxAltitude(obj, altitude)
            arguments
                obj
                altitude (1,1) double {mustBeNonnegative}
            end
            obj.maxAltitude = altitude;

            if ~isempty(obj.rngSeed)
                rng(obj.rngSeed);
            end

            obj.buildAltitudeGrid();
            obj.computeShearLaw();
            obj.computeTurbulence();
            obj.computeVeer();
            obj.computeWind3D();
        end

        %--------------------------------------------------------------
        % plotDiagnostics
        % Visualizes shear, veer, turbulence, and wind components.
        %
        % Useful for validating rhoTurb, k, I0, alpha, and stepSize.
        %--------------------------------------------------------------
        function plotDiagnostics(obj)
            figure;
        
            % --- Shear profile ---
            subplot(2,2,1);
            plot(obj.windMag, obj.alts, 'LineWidth', 2);
            xlabel('Wind Speed U(z) [m/s]');
            ylabel('Altitude [m]');
            title('Shear Law Profile');
            grid on;
        
            % --- Veer profile ---
            subplot(2,2,2);
            plot(rad2deg(obj.thetaVec), obj.alts, 'LineWidth', 2);
            xlabel('Direction θ(z) [deg]');
            ylabel('Altitude [m]');
            title('Veer Profile');
            grid on;
        
            % --- Turbulence amplitude ---
            subplot(2,2,3);
            plot(obj.sigmaVec, obj.alts, 'LineWidth', 2);
            xlabel('σ(z) [m/s]');
            ylabel('Altitude [m]');
            title('Turbulence Amplitude');
            grid on;
        
            % --- Wind components ---
            subplot(2,2,4);
            plot(obj.windVec3D(:,1), obj.alts); hold on;
            plot(obj.windVec3D(:,2), obj.alts);
            plot(obj.windVec3D(:,3), obj.alts);
            xlabel('Wind Component [m/s]');
            ylabel('Altitude [m]');
            title('Wind Components Wx, Wy, Wz');
            legend('Wx','Wy','Wz');
            grid on;
        end

    end

end