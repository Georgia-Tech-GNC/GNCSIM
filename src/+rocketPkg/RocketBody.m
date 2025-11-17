
classdef RocketBody < rocketPkg.DatcomManager
	% ROCKETBODY Represents the aerodynamic body of a rocket
	
	properties
		bodyParams			% Structure containing body geometry parameters
		bodyOutline		% [2 x n] array: row 1 = x stations, row 2 = body radii
						% DATCOM aerodynamic data
	end

	methods
		% Constructor
		function obj = RocketBody(bodyParams, flightConParams)
			% Call superclass constructor (required for inheritance)
			obj = obj@rocketPkg.DatcomManager(flightConParams)

			obj.bodyParams = bodyParams;
		end

        function [fB, mB] = getForcesMoments(obj, state, disturbances)
           	% Computes aerodynamic forces and moments on the body

           	% Rotation from inertial frame to body frame using quaternion
           	q = state(8:11);
            cBI = Utils.dirCosine(q);

            % Body frame velocity
            vB = state(5:7);

            % vW = cBI * disturbances.wind
            vW = cBI * [0;0;0]; % Wind velocity (ignored for now)

            vB = vB + vW;

			rCb = -obj.bodyParams.translation(:);
			
			% Component frame velocity and ang vel
			wB = state(12:14);

			vC = (vB + cross(wB, rCb));

			% Compute aerodynamic forces and moments using DATCOM data
			[fB, mC] = obj.getForcesMomentsDatcom(state, vC);

			mBRoss = cross(rCb, fB);
            mB = mC + mBRoss(:);
		end

		function rNose = getNoseRadii(obj, xNose)
			% Computes nose cone radius distribution

			L = obj.bodyParams.noseLength;
            R = obj.bodyParams.rocketDiameter / 2;
			
			% Compute the radius of curvature for the ogive
			rhoArc = (L^2 + R^2) / (2 * R);

			% Circle-based profile equation for an ogive nose
			rNose = sqrt(rhoArc^2 - (xNose - L).^2) + R - rhoArc + 0.0001;
		end

		function computeBuildup(obj)
			% Generates DATCOM input file for the rocket body

            nNosePoints = 10;   % Number of points defining the nose cone
            nBodyPoints = 5;    % Number of points defining the cylindrical body
            datcomFilename = 'BODY.dat';
			xcg = obj.bodyParams.cgXWet; 
			xw = 0;

			% Generate body geometry
            totalLength = obj.bodyParams.bodyLength + obj.bodyParams.noseLength;

			% Nose cone geometry
            xNose = linspace(0, obj.bodyParams.noseLength, nNosePoints);
			rNose = obj.getNoseRadii(xNose);

            % Cylindrical body geometry
            xBody = linspace(obj.bodyParams.noseLength, totalLength, nBodyPoints);
            rBody = ones(1, nBodyPoints) * obj.bodyParams.rocketDiameter / 2; % Radius is constant

            % Combine nose cone and body into continuous profile
            bodyStationsX = [xNose(1:end-1), xBody];
            bodyRadii      = [rNose(1:end-1), rBody];
            totalPoints    = length(bodyStationsX);
			obj.bodyOutline = [bodyStationsX; bodyRadii];

			% Open DATCOM input file and write sections
            obj.openDatcom(datcomFilename)
			obj.addFlightCondition()

			% Write $OPTINS Namelist
			R = obj.bodyParams.rocketDiameter / 2;
            fprintf(obj.fileID, ' $OPTINS SREF=%.4f, CBARR=%.4f, BLREF=%.4f$\n', pi*R^2, totalLength, 2*R);

            % Write $SYNTHS Namelist
            fprintf(obj.fileID, ' $SYNTHS XW=%.4f, XCG=%.4f$\n', xw, xcg);

            fprintf(obj.fileID, ' $BODY   NX=%.1f,\n', totalPoints);

            % Write the X-stations array
            fprintf(obj.fileID, ' X(1)=%.4f,', bodyStationsX(1));
            for i = 2:totalPoints
                % Add a newline every 8 values for readability, matching the example
                if mod(i-2, 8) == 0 && i > 2
                    fprintf(obj.fileID, '\n');
                end
                fprintf(obj.fileID, ' %.4f,', bodyStationsX(i));
			end
            fprintf(obj.fileID, '\n');

            % Write the Radii array
            fprintf(obj.fileID, ' R(1)=%.4f,', bodyRadii(1));
            for i = 2:totalPoints
                if mod(i-2, 8) == 0 && i > 2
                    fprintf(obj.fileID, '\n');
                end
                fprintf(obj.fileID, ' %.4f,', bodyRadii(i));
            end
            fseek(obj.fileID, -1, 'cof'); % Move cursor back one character
            fprintf(obj.fileID, '$\n');

			% Close DATCOM file
            obj.closeDatcom();

			% Run the .exe DATCOM file
			obj.runDatcom(datcomFilename)

			% Import DATCOM Data
            aeroCell = datcomimport('datcom.out');
            obj.aero = aeroCell{1};

			obj.buildInterps(obj.aero);
		end

		function drawBody(obj)
            % Draws a 3D rocket with a tangent ogive nose cone and cylindrical body.

            % Derived quantities
            r = obj.bodyParams.rocketDiameter/2;     % body radius
            xNose = linspace(0, obj.bodyParams.noseLength, 100);

            % Tangive Ogive profile
            yProfile = getNoseRadii(obj,xNose);

            % Add cylindrical body after nose
            xBody = linspace(obj.bodyParams.noseLength, obj.bodyParams.noseLength + obj.bodyParams.bodyLength, 20);
            yBody = r * ones(size(xBody));
            
            % Combine profiles
            x = [xNose, xBody];
            y = [yProfile, yBody];

            % Create 3D surface of revolution
            theta = linspace(0, 2*pi, 100);
            X = repmat(x(:), 1, 100);
            Y = repmat(y(:), 1, 100).*cos(theta);
            Z = repmat(y(:), 1, 100).*sin(theta);

            % Plot
            figure;
            surf(X, Y, Z, 'FaceColor', [0.8 0.8 0.9], 'EdgeColor', 'none');
            hold on;
            surf(X, -Y, Z, 'FaceColor', [0.8 0.8 0.9], 'EdgeColor', 'none'); % mirror
            axis equal;
            xlabel('Length (m)');
            ylabel('Radius (m)');
            zlabel('Z (m)');
            title(sprintf('Rocket: Nose Cone Length=%.2f, Rocket diameter=%.2f, Body length=%.2f', obj.bodyParams.noseLength, obj.bodyParams.rocketDiameter, obj.bodyParams.bodyLength));
            camlight headlight; lighting gouraud;
            grid on;
        end
	end
end

