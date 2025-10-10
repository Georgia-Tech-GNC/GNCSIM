classdef DatcomManager < handle
	properties
		fileID
	end
	methods
		function obj = DatcomManager()
		end

		function open_datcom(obj, datcom_filename)
			obj.fileID = fopen(datcom_filename, 'w');
            if obj.fileID == -1
                error('Could not open file for writing.');
			end
		end

		function add_flight_condition(obj, flight_con_params)
			mach_numbers = flight_con_params.mach_numbers;
            altitudes_ft = flight_con_params.altitudes_ft;
            alphas_deg   = flight_con_params.alphas_deg;
			
			if ~isempty(mach_numbers)
                fprintf(obj.fileID, ' $FLTCON NMACH=%.1f, MACH(1)=%.4f,\n', length(mach_numbers), mach_numbers(1));
                % Loop through the rest of the Mach numbers, printing each on a new line
                for i = 2:length(mach_numbers)
                    fprintf(obj.fileID, ' %.4f,\n', mach_numbers(i));
                end
            end
            
            
            if ~isempty(altitudes_ft)
                fprintf(obj.fileID, ' NALT=%.1f, ALT(1)=%.4f,\n', length(altitudes_ft), altitudes_ft(1));
                for i = 2:length(altitudes_ft)
                    fprintf(obj.fileID, ' %.4f,\n', altitudes_ft(i));
                end
            end
            
            if ~isempty(alphas_deg)
                fprintf(obj.fileID, ' NALPHA=%.1f, ALSCHD(1)=%.4f,\n', length(alphas_deg), alphas_deg(1));
                % Loop to the SECOND-TO-LAST element, printing a comma
                for i = 2:length(alphas_deg)
                    fprintf(obj.fileID, ' %.4f,\n', alphas_deg(i));
                end
            end
            fprintf(obj.fileID, ' LOOP=2.0$\n');
		end

		function close_datcom(obj)
			fclose(obj.fileID);
		end
	end	

	methods (Static)
		function run_datcom(datcom_filename)
			if ispc
                % --- Windows ---
                executable_name = 'DATCOM\datcom_win.exe';
                if exist(executable_name, 'file') ~= 2
                    error('"%s" not found in the current directory.', executable_name);
                end
                command = sprintf('echo %s | %s', datcom_filename, executable_name);
            elseif isunix
                disp("imagine being on mac. could not be me")
            end

            status = system(command);
            disp('DATCOM execution finished.');
		end
	end
end
