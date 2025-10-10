classdef RocketFins < rocket_pkg.DatcomManager
	properties
		fin_params
		num_fins
		fins  % array of fin objects
		fin_faces = [1, 2, 3, 4];
		fin_vertices
    end

	methods
        % Constructor
		function obj = RocketFins(fin_params)
			obj = obj@rocket_pkg.DatcomManager()
			obj.fin_params = fin_params;
			obj.num_fins = fin_params.num_fins;
            
            fin_params.angle = 0;
            main_fin = rocket_pkg.Fin("fin", true, fin_params);  % master fin
            obj.fins = main_fin;
            for ii=1:obj.num_fins - 1
                fin_params.angle = (360 / obj.num_fins) * ii;
                fin = rocket_pkg.Fin("fin", true, fin_params);
                obj.fins = [obj.fins, fin];
            end
		end

        function [F_c, M_c] = get_forces_and_moments(obj, state, disturbances)
            % get aero forces and moments in the body frame 
			F_c = [0, 0, 0].';
			M_c = [0, 0, 0].';
			for ii=1:obj.num_fins
				[~,~,F_c_fin, M_c_fin] = obj.fins(ii).get_forces_and_moments(state, disturbances);
				F_c = F_c + F_c_fin;
				M_c = M_c + M_c_fin;
			end
        end

        function compute_buildup(obj)
            root_chord  = obj.fin_params.root_chord;     % [m] Chord length at the wing root
            tip_chord   = obj.fin_params.tip_chord;      % [m] Chord length at the wing tip
            semi_span   = obj.fin_params.fin_span;       % [m] Span of one wing (from centerline to tip)
            sweep_deg   = obj.fin_params.sweep_deg;      % [deg] Leading-edge sweep angle
			translation = obj.fin_params.translation;    % [m] Refrance position in body frame
			wing_apex_x = 0;
            xcg = 0;

            flight_con_params.mach_numbers = [0.2, 0.4, 0.6];        % Array of Mach numbers to test
            flight_con_params.altitudes_ft = [0, 5000, 10000];       % Array of altitudes in feet
            flight_con_params.alphas_deg   = [-10, -8, -6, -4, -2, 0, 2, 4, 6, 8, 10]; % Array of angles of attack in degrees

            airfoil_naca = 'NACA 0012';
            datcom_filename = 'FIN.dat';

            % Convert angles to radians for MATLAB's trig functions
            sweep_rad = deg2rad(sweep_deg);
            dihedral_rad = 0;
            
            % Calculate the (x, y, z) coordinates for the 4 corners of the right wing
            % Point 1: Inboard Leading Edge
            v1 = [wing_apex_x, 0, 0];
            % Point 2: Inboard Trailing Edge
            v2 = [wing_apex_x + root_chord, 0, 0];
            % Point 3: Outboard Leading Edge
            x_tip_le = wing_apex_x + semi_span * tan(sweep_rad);
            z_tip = semi_span * tan(dihedral_rad);
            v3 = [x_tip_le, semi_span, z_tip];
            % Point 4: Outboard Trailing Edge
            v4 = [x_tip_le + tip_chord, semi_span, z_tip];
            
            % Assemble vertices and faces for the right wing patch
            obj.fin_vertices = [v1; v2; v4; v3];

            % Wing span (for a single panel, this is just the semi-span)
            blref = semi_span;
            
            % Taper ratio (lambda)
            taper_ratio = tip_chord / root_chord;
            
            % Wing area (area of one trapezoid)
            sref = semi_span * (root_chord + tip_chord) / 2;
            
            % Mean Aerodynamic Chord (MAC)
            cbarr = (2/3) * root_chord * (1 + taper_ratio + taper_ratio^2) / (1 + taper_ratio);
			
			obj.open_datcom(datcom_filename)
			
			obj.add_flight_condition(flight_con_params)
            
            % fileID = fopen(datcom_filename, 'w');
            % if fileID == -1
            %     error('Could not open file for writing.');
            % end
			% 
            % if ~isempty(mach_numbers)
            %     fprintf(fileID, ' $FLTCON NMACH=%.1f, MACH(1)=%.4f,\n', length(mach_numbers), mach_numbers(1));
            %     % Loop through the rest of the Mach numbers, printing each on a new line
            %     for i = 2:length(mach_numbers)
            %         fprintf(fileID, ' %.4f,\n', mach_numbers(i));
            %     end
            % end
			% 
            % % --- Write subsequent variables ---
            % % Note: These do NOT start with '$FLTCON'
            % if ~isempty(altitudes_ft)
            %     fprintf(fileID, ' NALT=%.1f, ALT(1)=%.4f,\n', length(altitudes_ft), altitudes_ft(1));
            %     for i = 2:length(altitudes_ft)
            %         fprintf(fileID, ' %.4f,\n', altitudes_ft(i));
            %     end
            % end
			% 
            % if ~isempty(alphas_deg)
            %     fprintf(fileID, ' NALPHA=%.1f, ALSCHD(1)=%.4f,\n', length(alphas_deg), alphas_deg(1));
            %     % Loop to the SECOND-TO-LAST element, printing a comma
            %     for i = 2:length(alphas_deg)
            %         fprintf(fileID, ' %.4f,\n', alphas_deg(i));
            %     end
            % end
            % fprintf(fileID, ' LOOP=2.0$\n');
            % --- Write $OPTINS Namelist ---
            fprintf(obj.fileID, ' $OPTINS SREF=%.4f, CBARR=%.4f, BLREF=%.4f$\n', sref, cbarr, blref);
            
            % --- Write $SYNTHS Namelist ---
            fprintf(obj.fileID, ' $SYNTHS XW=%.4f, XCG=%.4f$\n', wing_apex_x, xcg);
            
            % --- Write $WGPLNF Namelist ---
            fprintf(obj.fileID, ' $WGPLNF CHRDR=%.4f, CHRDTP=%.4f, SSPN=%.4f, SSPNE=%.4f,\n', ...
                root_chord, tip_chord, semi_span, semi_span);
            fprintf(obj.fileID, ' SAVSI=%.4f, DHDADI=%.4f, TYPE=1.0$\n', sweep_deg, 0);

			fprintf(obj.fileID, 'NACA W 4 %s\n', airfoil_naca);
            
            % --- Write NACA Airfoil Card ---
            % Parse the airfoil string to fit DATCOM format
            % parts = strsplit(airfoil_naca);
            % if length(parts) == 2 && strcmpi(parts{1}, 'NACA') && (length(parts{2}) == 4 || length(parts{2}) == 5)
            %     if length(parts{2}) == 4
            %         naca_series = '4';
            %     else
            %         naca_series = '5';
            %     end
            %     naca_profile = parts{2};
            %     fprintf(fileID, 'NACA W %s %s\n', naca_series, naca_profile);
            % else
            %     warning('Airfoil format not recognized. Writing a default NACA 0012.');
            %     fprintf(fileID, 'NACA W 4 0012\n');
            % end
            
            % --- Write Case Control ---
            fprintf(obj.fileID, 'CASEID GENERATED WING-ONLY ANALYSIS\n');
            fprintf(obj.fileID, 'NEXT CASE\n');
            
            % --- Close the file ---
            obj.close_datcom();
            
            disp('DATCOM file successfully generated!');

			obj.run_datcom(datcom_filename)
            
            aero_cell = datcomimport('datcom.out');
            aero = aero_cell{1};

            for ii=1:length(obj.fins)
                obj.fins(ii).aero = aero;
            end
		end

		function draw_fin(obj)
			figure('Name', 'Fin Geometry');
			hold on;

			plot([obj.fin_vertices(:, 1); obj.fin_vertices(1, 1)], [obj.fin_vertices(:, 2); obj.fin_vertices(1, 2)], 'k-')
			
			% Formatting the plot
			title('Generated Wing Geometry');
			xlabel('X - Longitudinal Axis (m)');
			ylabel('Y - Spanwise Axis (m)');
			%zlabel('Z - Vertical Axis (ft)');
			grid on;
			axis equal;
			%view(30, 25);
			hold off;
		end
	end
end

