classdef DatcomManager < handle
	% DATCOMMANAGER Base class for managing DATCOM file generation,
	% execution, and aerodynamic data import. Provides functionality for
	% creating input files, defining flight conditions, and running DATCOM.

	properties
		fileID				% File identifier for the currently open DATCOM input file
		flight_con_params	% Structure containing flight condition parameters (Mach, altitude, alpha)
	end

	methods
		% Constructor
		function obj = DatcomManager(flight_con_params)
			% Initializes the DatcomManager object and stores flight condition parameters
			obj.flight_con_params = flight_con_params;
		end

		function open_datcom(obj, datcom_filename)
			% Opens a new DATCOM input file for writing.
			% Throws an error if the file cannot be opened.
			obj.fileID = fopen(datcom_filename, 'w');
            if obj.fileID == -1
                error('Could not open file for writing.');
			end
		end

		function add_flight_condition(obj)
			% Writes the $FLTCON namelist to the DATCOM input file.
			% Includes Mach numbers, altitudes (m), and angle-of-attack schedule.
			mach_numbers = obj.flight_con_params.mach_numbers;
            altitudes = obj.flight_con_params.altitudes;
            alphas_deg   = obj.flight_con_params.alphas_deg;
			
			% Write Mach numbers
			if ~isempty(mach_numbers)
                fprintf(obj.fileID, ' $FLTCON NMACH=%.1f, MACH(1)=%.4f,\n', length(mach_numbers), mach_numbers(1));
                % Loop through the rest of the Mach numbers, printing each on a new line
                for i = 2:length(mach_numbers)
                    fprintf(obj.fileID, ' %.4f,\n', mach_numbers(i));
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
            if ~isempty(alphas_deg)
                fprintf(obj.fileID, ' NALPHA=%.1f, ALSCHD(1)=%.4f,\n', length(alphas_deg), alphas_deg(1));
                % Loop to the SECOND-TO-LAST element, printing a comma
                for i = 2:length(alphas_deg)
                    fprintf(obj.fileID, ' %.4f,\n', alphas_deg(i));
                end
			end
			% End of flight condition block
            fprintf(obj.fileID, ' LOOP=2.0$\n');
        end

		function close_datcom(obj)
			% Closes the DATCOM input file

			% Set units to meters
			fprintf(obj.fileID, 'DIM M\n');
			fprintf(obj.fileID, 'NEXT CASE\n');
			fclose(obj.fileID);
		end
	end	

	methods (Static)
		function run_datcom(datcom_filename)
			% Executes the DATCOM program with the specified input file

			if ispc
                % --- Windows ---
                executable_name = 'DATCOM\datcom_win.exe';
                if exist(executable_name, 'file') ~= 2
                    error('"%s" not found in the current directory.', executable_name);
                end
                command = sprintf('echo %s | %s', datcom_filename, executable_name);
            elseif isunix
				% --- Unix / macOS ---
                disp("imagine being on mac. could not be me.")
            end
			
			% Run DATCOM and display completion message
            status = system(command);
            disp('DATCOM execution finished.');
        end

        function [F_c, M_c] = get_forces_and_moments(state, v_c, aero)
			
			% Aerodynamic coefficients
           	v_a = sqrt(v_c(1)^2+v_c(2)^2+v_c(3)^2);
           	a = 343; % Speed of sound approximation (TODO: Replace with better approximation)
           	M = v_a/a;
           	AoA = atan2d(v_c(3),v_c(1));
	
           	pos = state(2:4);
           	altitude = pos(3);  % z componet of position
           	[ca, cm, cn] = rocket_pkg.DatcomManager.aero_interp(aero, M, AoA, altitude);
	
           	S = aero.sref;
           	c = aero.cbar;
			% TODO change rho (density) to function of height
			rho = 1.225;
           	q = 0.5 * v_a^2 * rho;  
	
           	% Forces + Moments in component frame
           	N = cn*q*S;
           	A = ca*q*S;
           	M = cm*q*c;
           	F_c = [-A; 0; -N];
		   	M_c = [0; M; 0];
		end

		function [ca, cm, cn] = aero_interp(aero, mach, AoA_deg, alt)
			% TODO change this function to generate functions
            ca_F = griddedInterpolant({aero.alpha(:), aero.mach(:), aero.alt(:)}, ...
                        aero.ca, "linear");
            ca = ca_F(AoA_deg, mach, alt);

            cm_F = griddedInterpolant({aero.alpha(:), aero.mach(:), aero.alt(:)}, ...
                        aero.cm, "linear");
            cm = cm_F(AoA_deg, mach, alt);
            
            cn_F = griddedInterpolant({aero.alpha(:), aero.mach(:), aero.alt(:)}, ...
                        aero.cn, "linear");
			cn = cn_F(AoA_deg, mach, alt);

			alpha_range = [min(aero.alpha(:)), max(aero.alpha(:))];
    		mach_range  = [0, max(aero.mach(:))];
    		alt_range   = [min(aero.alt(:)),  max(aero.alt(:))];
		
    		% Check bounds
    		if AoA_deg < alpha_range(1) || AoA_deg > alpha_range(2)
        		warning('AoA %.2f° is out of range [%.2f, %.2f].', AoA_deg, alpha_range(1), alpha_range(2));
    		end
    		if mach < mach_range(1) || mach > mach_range(2)
        		warning('Mach %.2f is out of range [%.2f, %.2f].', mach, mach_range(1), mach_range(2));
    		end
    		if alt < alt_range(1) || alt > alt_range(2)
        		warning('Altitude %.2f m is out of range [%.2f, %.2f].', alt, alt_range(1), alt_range(2));
    		end
        end
    end
end

