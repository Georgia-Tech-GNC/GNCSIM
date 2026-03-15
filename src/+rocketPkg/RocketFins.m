% Main Authors: David Reynolds (dreynolds45@gatech.edu)

classdef RocketFins < rocketPkg.DatcomManager
	% ROCKETFINS Manages the geometry, forces, and moments for a cluster
	% of equally-spaced fins on a rocket. Inherits from DatcomManager
	% to facilitate DATCOM analysis.

	properties
		finParamsArr		    % Array of structs containing fin geometry parameters
		fins			        % Array of individual rocket_pkg.Fin objects
		finVertices		        % (4x3 array) (x, y, z) coordinates of the single reference fin's vertices
        numFins                 % Number of Fins in set
        numMovableFins          % Number of Fins in set that can deflect
        numMovableIndex = []    % The index of which fins can deflect
    end

	methods
        % Constructor
		function obj = RocketFins(finParamsArr, flightConParams)
			% Call superclass constructor (required for inheritance)
			obj = obj@rocketPkg.DatcomManager(flightConParams)
			obj.finParamsArr = finParamsArr;
            
            % Loop over each fin set
			for ii=1:length(finParamsArr)
                finParams = finParamsArr(ii);
                translation = finParams.translation;
                name = finParams.name;

                % Create and space the individual Fin objects
                finParams.angle = 0;
                mainFin = rocketPkg.Fin(name, finParams, flightConParams);  % master fin
                obj.fins = [obj.fins, mainFin];

                numFins = finParams.numFins;
    
                for jj=1:numFins - 1
				    % Calculate angle for current fin
                    finParams.angle = (360 / numFins) * jj;
				    finParams.translation = Utils.finDcm(0, finParams.angle) * translation(:);
    
                    fin = rocketPkg.Fin(name, finParams, flightConParams);
                    obj.fins = [obj.fins, fin];
                end
            end

            % Get number of fins
            obj.numFins = length(obj.fins);
            
            % Get number of movable fins
            numMovableFins = 0;
            for ii=1:obj.numFins
                if obj.fins(ii).finParams.movable == true
                    numMovableFins = numMovableFins + 1;
                    obj.numMovableIndex = [obj.numMovableIndex, ii];
                end
            end
            obj.numMovableFins = numMovableFins;
		end

        function [fB, mB] = getForcesMoments(obj, state, deflections, currentCg, disturbances)
            % Calculates the total aerodynamic forces and moments in the body frame
			fB = [0, 0, 0].';
			mB = [0, 0, 0].';

			% Sum contributions from all individual fins
			for ii=1:length(obj.fins)
                fin = obj.fins(ii);
                deflection = 0;

                % if the fin is movable then use the deflection in the array
                if ismember(ii, obj.numMovableIndex)
                    idx = ii == obj.numMovableIndex;
                    deflection = deflections(idx);
                end
                
                % Compute the current fin forces and moments based on
                % state, current deflection, cg location and any
                % disterbances
				[fBFin, mBFin] = fin.getForcesMoments(state, deflection, currentCg, disturbances);
				fB = fB + fBFin;
				mB = mB + mBFin;
			end
        end

        function computeBuildup(obj)
			% Generates the DATCOM input file, runs the executable, and loads the data
            
            for ii=1:length(obj.finParamsArr)
                finParams = obj.finParamsArr(ii);
                
			    % Geometry Setup
                rootChord  = finParams.rootChord;     % [m] Chord length at the wing root
                tipChord   = finParams.tipChord;      % [m] Chord length at the wing tip
                semiSpan   = finParams.finSpan;       % [m] Span of one wing (from centerline to tip)
                sweepDeg   = finParams.sweepDeg;      % [deg] Leading-edge sweep angle
                name       = finParams.name;
                xcg = 0;  % Keep at zero
    
                airfoilNaca = 'NACA 0012';
                datcomFilename = [name, '.dat'];
                datcomFilePath = [tempname, '_', datcomFilename];
                cleanupDeck = onCleanup(@() rocketPkg.DatcomManager.deleteIfExists(datcomFilePath)); %#ok<NASGU>
    
                % Convert angles to radians for MATLAB's trig functions
                sweepRad = deg2rad(sweepDeg);
    
			    % Calculate Reference Geometry
                obj.calculateRefGeometry(rootChord, tipChord, semiSpan, sweepRad)
    
                % Wing span (for a single panel, this is just the semi-span)
                blref = semiSpan;
    
                % Taper ratio (lambda)
                taperRatio = tipChord / rootChord;
    
                % Wing area (area of one trapezoid)
                sref = semiSpan * (rootChord + tipChord) / 2;
    
                % Mean Aerodynamic Chord (MAC)
                cbarr = (2/3) * rootChord * (1 + taperRatio + taperRatio^2) / (1 + taperRatio);
    
			    % Open DATCOM file
			    obj.openDatcom(datcomFilePath)
    
			    % Adds flight conditions section into DATCOM file
			    obj.addFlightCondition()
    
                % Write $OPTINS Namelist
                fprintf(obj.fileID, ' $OPTINS SREF=%.4f, CBARR=%.4f, BLREF=%.4f$\n', sref, cbarr, blref);
    
                % Write $SYNTHS Namelist
                fprintf(obj.fileID, ' $SYNTHS XW=%.4f, XCG=%.4f$\n', 0, xcg);
    
                % Write $WGPLNF Namelist
                fprintf(obj.fileID, ' $WGPLNF CHRDR=%.4f, CHRDTP=%.4f, SSPN=%.4f, SSPNE=%.4f,\n', ...
                    rootChord, tipChord, semiSpan, semiSpan);
    
			    % Write sweep angle and dihedral angle (should be zero)
                fprintf(obj.fileID, ' SAVSI=%.4f, DHDADI=%.4f, TYPE=1.0$\n', sweepDeg, 0);
    
			    % Write NACA airfoil type (should be symetric
			    fprintf(obj.fileID, 'NACA W 4 %s\n', airfoilNaca);
    
                % Close DATCOM file
                obj.closeDatcom();
                obj.lastDatcomDeckText = fileread(datcomFilePath);
    
                aero = obj.resolveAeroData(datcomFilePath);

                for jj=1:length(obj.fins)
                    if isequal(obj.fins(jj).name, finParams.name)
                        obj.fins(jj).aero = aero;
                        obj.fins(jj).buildInterps(aero);
                    end
                end
            end
		end

		function calculateRefGeometry(obj, rootChord, tipChord, semiSpan, sweepRad)
			% Calculate Reference Geometry
			v1 = [0, 0, 0];
            v2 = [rootChord, 0, 0];
            xTipLe = semiSpan * tan(sweepRad);
            v3 = [xTipLe, semiSpan, 0];
            v4 = [xTipLe + tipChord, semiSpan, 0];

			% Assemble vertices and faces for the right wing patch
            obj.finVertices = [v1; v2; v4; v3];
		end

		function drawFin(obj)
            leRoot = [0, 0];
            leTip  = [obj.finParams.finSpan * tand(obj.finParams.sweepDeg), obj.finParams.finSpan];
            teRoot = [obj.finParams.rootChord, 0];
            teTip  = [leTip(1) + obj.finParams.tipChord, obj.finParams.finSpan];

            % Create arrays for plotting
            x = [leRoot(1), teRoot(1), teTip(1), leTip(1), leRoot(1)];
            y = [leRoot(2), teRoot(2), teTip(2), leTip(2), leRoot(2)];

            % Plot the fin
            figure;
            fill(x, y, [0.2 0.6 0.9], 'EdgeColor', 'k', 'LineWidth', 1.5);
            axis equal;
            xlabel('Chord direction (m)');
            ylabel('Span direction (m)');
            title(sprintf('Fin Geometry: Root chord=%.2f, Tip chord=%.2f, Fin span=%.2f, Sweep angle=%.1f°', obj.finParams.rootChord, obj.finParams.tipChord, obj.finParams.finSpan, obj.finParams.sweepDeg));
            grid on;
		end
	end
end
