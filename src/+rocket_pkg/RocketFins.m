classdef RocketFins < rocket_pkg.DatcomManager
	% ROCKETFINS Manages the geometry, forces, and moments for a cluster
	% of equally-spaced fins on a rocket. Inherits from DatcomManager
	% to facilitate DATCOM analysis.

	properties
		fin_params		% Structure containing fin geometry parameters
		num_fins		% Total number of fins
		fins			% Array of individual rocket_pkg.Fin objects
		fin_vertices	% (4x3 array) (x, y, z) coordinates of the single reference fin's vertices
    end

	methods
        % Constructor
		function obj = RocketFins(fin_params, flight_con_params)
			% Call superclass constructor (required for inheritance)
			obj = obj@rocket_pkg.DatcomManager(flight_con_params)

			obj.fin_params = fin_params;
			obj.num_fins = fin_params.num_fins;
			translation = fin_params.translation;
            
			% Create and space the individual Fin objects
            fin_params.angle = 0;
            main_fin = rocket_pkg.Fin("fin", fin_params);  % master fin
            obj.fins = main_fin;

            for ii=1:obj.num_fins - 1

				% Calculate angle for current fin
                fin_params.angle = (360 / obj.num_fins) * ii;
				fin_params.translation = Utils.fin_dcm(0, fin_params.angle) * translation(:);

                fin = rocket_pkg.Fin("fin", fin_params);
                obj.fins = [obj.fins, fin];
            end
		end

        function [F_b, M_b] = get_forces_and_moments(obj, state, disturbances)
            % Calculates the total aerodynamic forces and moments in the body frame 
			F_b = [0, 0, 0].';
			M_b = [0, 0, 0].';

			% Sum contributions from all individual fins
			for ii=1:obj.num_fins
				[F_b_fin, M_b_fin] = obj.fins(ii).get_forces_and_moments(state, disturbances);
				F_b = F_b + F_b_fin;
				M_b = M_b + M_b_fin;
			end
        end

        function compute_buildup(obj)
			% Generates the DATCOM input file, runs the executable, and loads the data
			
			% Geometry Setup
            root_chord  = obj.fin_params.root_chord;     % [m] Chord length at the wing root
            tip_chord   = obj.fin_params.tip_chord;      % [m] Chord length at the wing tip
            semi_span   = obj.fin_params.fin_span;       % [m] Span of one wing (from centerline to tip)
            sweep_deg   = obj.fin_params.sweep_deg;      % [deg] Leading-edge sweep angle
			translation = obj.fin_params.translation;    % [m] Refrance position in body frame
            xcg = 0;  % Keep at zero

            airfoil_naca = 'NACA 0012';
            datcom_filename = 'FIN.dat';

            % Convert angles to radians for MATLAB's trig functions
            sweep_rad = deg2rad(sweep_deg);
            
			% Calculate Reference Geometry
            obj.calculate_ref_geometry(root_chord, tip_chord, semi_span, sweep_rad)

            % Wing span (for a single panel, this is just the semi-span)
            blref = semi_span;
            
            % Taper ratio (lambda)
            taper_ratio = tip_chord / root_chord;
            
            % Wing area (area of one trapezoid)
            sref = semi_span * (root_chord + tip_chord) / 2;
            
            % Mean Aerodynamic Chord (MAC)
            cbarr = (2/3) * root_chord * (1 + taper_ratio + taper_ratio^2) / (1 + taper_ratio);
			
			% Open DATCOM file
			obj.open_datcom(datcom_filename)
			
			% Adds flight conditions section into DATCOM file
			obj.add_flight_condition()
            
            % Write $OPTINS Namelist
            fprintf(obj.fileID, ' $OPTINS SREF=%.4f, CBARR=%.4f, BLREF=%.4f$\n', sref, cbarr, blref);
            
            % Write $SYNTHS Namelist
            fprintf(obj.fileID, ' $SYNTHS XW=%.4f, XCG=%.4f$\n', 0, xcg);
            
            % Write $WGPLNF Namelist
            fprintf(obj.fileID, ' $WGPLNF CHRDR=%.4f, CHRDTP=%.4f, SSPN=%.4f, SSPNE=%.4f,\n', ...
                root_chord, tip_chord, semi_span, semi_span);

			% Write sweep angle and dihedral angle (should be zero) 
            fprintf(obj.fileID, ' SAVSI=%.4f, DHDADI=%.4f, TYPE=1.0$\n', sweep_deg, 0);
			
			% Write NACA airfoil type (should be symetric
			fprintf(obj.fileID, 'NACA W 4 %s\n', airfoil_naca);
            
            % Close DATCOM file
            obj.close_datcom();
            
			% Run the .exe DATCOM file
			obj.run_datcom(datcom_filename)
            
			% Import DATCOM Data
            aero_cell = datcomimport('datcom.out');
            aero = aero_cell{1};
			
			% Takes DATCOM aero data and adds it to each fin object
            for ii=1:length(obj.fins)
                obj.fins(ii).aero = aero;
            end
		end

		function calculate_ref_geometry(obj, root_chord, tip_chord, semi_span, sweep_rad)
			% Calculate Reference Geometry
			v1 = [0, 0, 0];
            v2 = [root_chord, 0, 0];
            x_tip_le = semi_span * tan(sweep_rad);
            v3 = [x_tip_le, semi_span, 0];
            v4 = [x_tip_le + tip_chord, semi_span, 0];

			% Assemble vertices and faces for the right wing patch
            obj.fin_vertices = [v1; v2; v4; v3];
		end

		function draw_fin(obj)
            LE_root = [0, 0];
            LE_tip  = [obj.fin_params.fin_span * tand(obj.fin_params.sweep_deg), obj.fin_params.fin_span];
            TE_root = [obj.fin_params.root_chord, 0];
            TE_tip  = [LE_tip(1) + obj.fin_params.tip_chord, obj.fin_params.fin_span];
            
            % Create arrays for plotting
            x = [LE_root(1), TE_root(1), TE_tip(1), LE_tip(1), LE_root(1)];
            y = [LE_root(2), TE_root(2), TE_tip(2), LE_tip(2), LE_root(2)];
            
            % Plot the fin
            figure;
            fill(x, y, [0.2 0.6 0.9], 'EdgeColor', 'k', 'LineWidth', 1.5);
            axis equal;
            xlabel('Chord direction (m)');
            ylabel('Span direction (m)');
            title(sprintf('Fin Geometry: Root chord=%.2f, Tip chord=%.2f, Fin span=%.2f, Sweep angle=%.1f°', obj.fin_params.root_chord, obj.fin_params.tip_chord, obj.fin_params.fin_span, obj.fin_params.sweep_deg));
            grid on;
		end
	end
end

