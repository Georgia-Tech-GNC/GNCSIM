classdef DatcomManager < handle
	% DATCOMMANAGER Base class for managing DATCOM file generation,
	% execution, and aerodynamic data import. Provides functionality for
	% creating input files, defining flight conditions, and running DATCOM.

	properties
		fileID				% File identifier for the currently open DATCOM input file
		flightConParams	% Structure containing flight condition parameters (Mach, altitude, alpha)
	end

	methods
		% Constructor
		function obj = DatcomManager(flightConParams)
			% Initializes the DatcomManager object and stores flight condition parameters
			obj.flightConParams = flightConParams;
		end

		function openDatcom(obj, datcomFilename)
			% Opens a new DATCOM input file for writing.
			% Throws an error if the file cannot be opened.
			obj.fileID = fopen(datcomFilename, 'w');
            if obj.fileID == -1
                error('Could not open file for writing.');
			end
		end

		function addFlightCondition(obj)
			% Writes the $FLTCON namelist to the DATCOM input file.
			% Includes Mach numbers, altitudes (m), and angle-of-attack schedule.
			machNumbers = obj.flightConParams.machNumbers;
            altitudes = obj.flightConParams.altitudes;
            alphasDeg   = obj.flightConParams.alphasDeg;
			
			% Write Mach numbers
			if ~isempty(machNumbers)
                fprintf(obj.fileID, ' $FLTCON NMACH=%.1f, MACH(1)=%.4f,\n', length(machNumbers), machNumbers(1));
                % Loop through the rest of the Mach numbers, printing each on a new line
                for i = 2:length(machNumbers)
                    fprintf(obj.fileID, ' %.4f,\n', machNumbers(i));
                end
            end

            % Write altitudes
            if ~isempty(altitudes)
                fprintf(obj.fileID, ' NALT=%.1f, ALT(1)=%.4f,\n', length(altitudes), altitudes(1));
                for i = 2:length(altitudes)
                    fprintf(obj.fileID, ' %.4f,\n', altitudes(i));
                end
            end

			% Write angle-of-attacks
            if ~isempty(alphasDeg)
                fprintf(obj.fileID, ' NALPHA=%.1f, ALSCHD(1)=%.4f,\n', length(alphasDeg), alphasDeg(1));
                % Loop to the SECOND-TO-LAST element, printing a comma
                for i = 2:length(alphasDeg)
                    fprintf(obj.fileID, ' %.4f,\n', alphasDeg(i));
                end
			end
			% End of flight condition block
            fprintf(obj.fileID, ' LOOP=2.0$\n');
        end

		function closeDatcom(obj)
			% Closes the DATCOM input file

			% Set units to meters
			fprintf(obj.fileID, 'DIM M\n');
			fprintf(obj.fileID, 'NEXT CASE\n');
			fclose(obj.fileID);
		end
	end

	methods (Static)
		function runDatcom(datcomFilename)
			% Executes the DATCOM program with the specified input file

			if ispc
                % --- Windows ---
                executableName = 'DATCOM\datcom_win.exe';
                if exist(executableName, 'file') ~= 2
                    error('"%s" not found in the current directory.', executableName);
                end
                command = sprintf('echo %s | %s', datcomFilename, executableName);
            elseif isunix
				% --- Unix / macOS ---
                disp("imagine being on mac. could not be me.")
            end

			% Run DATCOM and display completion message
            status = system(command);
            disp('DATCOM execution finished.');
        end

        function [fC, mC] = getForcesMoments(state, vC, aero)

			% Aerodynamic coefficients
           	vA = sqrt(vC(1)^2+vC(2)^2+vC(3)^2);
           	a = 343; % Speed of sound approximation (TODO: Replace with better approximation)
           	M = vA/a;
           	AoA = atan2d(vC(3),vC(1));

           	pos = state(2:4);
           	altitude = pos(3);  % z componet of position
           	[ca, cm, cn] = rocketPkg.DatcomManager.aeroInterp(aero, M, AoA, altitude);
	
           	S = aero.sref;
           	c = aero.cbar;
			% TODO change rho (density) to function of height
			rho = 1.225;
           	q = 0.5 * vA^2 * rho;

           	% Forces + Moments in component frame
           	N = cn*q*S;
           	A = ca*q*S;
           	M = cm*q*c;
           	fC = [-A; 0; -N];
		   	mC = [0; M; 0];
		end

		function [ca, cm, cn] = aeroInterp(aero, mach, aoaDeg, alt)
			% TODO change this function to generate functions
            caF = griddedInterpolant({aero.alpha(:), aero.mach(:), aero.alt(:)}, ...
                        aero.ca, "linear");
            ca = caF(aoaDeg, mach, alt);

            cmF = griddedInterpolant({aero.alpha(:), aero.mach(:), aero.alt(:)}, ...
                        aero.cm, "linear");
            cm = cmF(aoaDeg, mach, alt);

            cnF = griddedInterpolant({aero.alpha(:), aero.mach(:), aero.alt(:)}, ...
                        aero.cn, "linear");
			cn = cnF(aoaDeg, mach, alt);

			alphaRange = [min(aero.alpha(:)), max(aero.alpha(:))];
    		machRange  = [0, max(aero.mach(:))];
    		altRange   = [min(aero.alt(:)),  max(aero.alt(:))];

    		% Check bounds
    		if aoaDeg < alphaRange(1) || aoaDeg > alphaRange(2)
        		warning('AoA %.2f° is out of range [%.2f, %.2f].', aoaDeg, alphaRange(1), alphaRange(2));
    		end
    		if mach < machRange(1) || mach > machRange(2)
        		warning('Mach %.2f is out of range [%.2f, %.2f].', mach, machRange(1), machRange(2));
    		end
    		if alt < altRange(1) || alt > altRange(2)
        		warning('Altitude %.2f m is out of range [%.2f, %.2f].', alt, altRange(1), altRange(2));
    		end
        end
    end
end

