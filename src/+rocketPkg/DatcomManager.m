% Main Authors: David Reynolds (dreynolds45@gatech.edu)
%               Kandarp Vadia (kvadia3@gatech.edu)

classdef DatcomManager < handle
	% DATCOMMANAGER Base class for managing DATCOM file generation,
	% execution, and aerodynamic data import. Provides functionality for
	% creating input files, defining flight conditions, and running DATCOM.

	properties (Constant)
		idx = StateIndices
	end

	properties
		fileID				% File identifier for the currently open DATCOM input file
		flightConParams		% Structure containing flight condition parameters (Mach, altitude, alpha)
		caInterp
		cmInterp
		cnInterp
		aero
		scaleFactor = 1;
		momentScaleFactor = 1;
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

		function buildInterps(obj, aero)
            alpha = unique(aero.alpha(:));
            mach = unique(aero.mach(:));
            alt = unique(aero.alt(:));
            
            obj.caInterp = griddedInterpolant({alpha, mach, alt}, aero.ca, "linear");
            obj.cmInterp = griddedInterpolant({alpha, mach, alt}, aero.cm, "linear");
            obj.cnInterp = griddedInterpolant({alpha, mach, alt}, aero.cn, "linear");
        end

		function [ca, cm, cn] = aeroInterp(obj, mach, aoaDeg, alt)
			% TODO change this function to generate functions

            ca = obj.scaleFactor * obj.caInterp(aoaDeg, mach, alt);
            cm = obj.momentScaleFactor * obj.cmInterp(aoaDeg, mach, alt);
			cn = obj.scaleFactor * obj.cnInterp(aoaDeg, mach, alt);

			alphaRange = [min(obj.aero.alpha(:)), max(obj.aero.alpha(:))];
    		machRange  = [0, max(obj.aero.mach(:))];
    		altRange   = [min(obj.aero.alt(:)),  max(obj.aero.alt(:))];

    		% Check bounds
            if 0
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

		function [fC, mC] = getForcesMomentsDatcomBody(obj, state, vC)
			% Aerodynamic coefficients
           	vA = sqrt(vC(1)^2+vC(2)^2+vC(3)^2);
           	alpha = 343; % Speed of sound approximation (TODO: Replace with better approximation)
           	M = vA/alpha;
           	AoAz = atan2d(vC(3), vC(1));  % main AoA
			AoAy = atan2d(vC(2), vC(1));  % AoA for y axis (very similar to sideslip)

			% compute total angle of attack
			totalAoA = acosd(cosd(AoAz) * cosd(AoAy));

			theta = atan2(vC(2), vC(3) + 1e-6);
			
			Rx = [
				1,           0,          0;
				0,           cos(theta), sin(theta);
				0,           -sin(theta), cos(theta)
			];
			
			% calculate coeffients in wind frame
			pos = state(obj.idx.pos);
           	altitude = pos(3);  % z componet of position
			[ca, cm, cn] = obj.aeroInterp(M, totalAoA, altitude);
	
           	S = obj.aero.sref;
           	c = obj.aero.cbar;
			% TODO change rho (density) to function of height
			rho = 1.225;
           	q = 0.5 * vA^2 * rho;

			% Forces + Moments in component frame
           	N = cn*q*S;
           	A = ca*q*S;
           	M = cm*q*c;
           	fW = [-A; 0; -N];
		   	mW = [0; M; 0];
			
			% Rotate forces and moments back to componet frame
			fC = Rx * fW;
			mC = Rx * mW;
		end

		function [fC, mC] = getForcesMomentsDatcom(obj, state, vC)

			% Aerodynamic coefficients
           	vA = sqrt(vC(1)^2+vC(2)^2+vC(3)^2);
           	alpha = 343; % Speed of sound approximation (TODO: Replace with better approximation)
           	M = vA/alpha;
           	AoA = atan2d(vC(3),vC(1));  % main AoA

           	pos = state(obj.idx.pos);
           	altitude = pos(3);  % z componet of position
           	[ca, cm, cn] = obj.aeroInterp(M, AoA, altitude);
	
           	S = obj.aero.sref;
           	c = obj.aero.cbar;
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
            elseif isunix || ismac
				% --- Unix / macOS ---
                % Using fullfile ensures the path works across different shells
                executableName = fullfile('.', 'DATCOM/datcom_unix'); 
                
                if exist(executableName, 'file') ~= 2
                    error('"%s" not found. Did you forget to compile it with gfortran?', executableName);
                end
                
                executableName = './DATCOM/datcom_unix';
    
                % Force the input file to be 'FOR005'
                copyfile(datcomFilename, 'for005');
                
                % Pipe the filename string INTO the program
                command = sprintf('echo "for005\n" | %s', executableName);
                
                [status, cmdout] = system(command);
               
            else
                error('Unsupported Operating System.');
            end
            
            % Cleanup
            if exist('FOR006', 'file')
                movefile('FOR006', 'datcom.out');
            elseif exist('for006.dat', 'file')
                movefile('for006.dat', 'datcom.out');
            end
            
            % Identify all other potential 'FORxxx' files
            % This handles both FOR005, for0013, etc.
            leftovers = [dir('FOR0*'); dir('for0*'); dir('*.PTR'); dir('*.SRF')];
            
            % Delete them all
            for k = 1:length(leftovers)
                fileName = leftovers(k).name;
                % Don't delete the file you just saved as datcom.out
                if ~strcmpi(fileName, 'datcom.out') && ~strcmpi(fileName, datcomFilename)
                    delete(fileName);
                end
            end

			% Run DATCOM and display completion message
            status = system(command);
            disp('DATCOM execution finished.');
		end
    end
end

