

clear; clc; close all;

% USER-DEFINED INPUTS
% --- Wing Geometry ---
root_chord  = 0.203;      % [m] Chord length at the wing root
tip_chord   = 0.17;      % [m] Chord length at the wing tip
semi_span   = 0.144;      % [m] Span of one wing (from centerline to tip)
sweep_deg   = 15.0;     % [deg] Leading-edge sweep angle
dihedral_deg = 0.0;     % [deg] Dihedral angle (upward angle of the wings)
wing_apex_x = 0.0;      % [m] X-location of the wing's leading edge at the root

% --- Airfoil ---
% Must be a string in a format like 'NACA 0012' or 'NACA 2415'
airfoil_naca = 'NACA 0012';

% --- Flight Conditions ---
mach_numbers = [0.2, 0.4, 0.6];  % Array of Mach numbers to test
altitudes_ft = [0, 5000, 10000];       % Array of altitudes in feet
alphas_deg   = [-10, -8, -6, -4, -2, 0, 2, 4, 6, 8, 10]; % Array of angles of attack in degrees
%alphas_deg = 1:10;

% --- Center of Gravity ---
% The longitudinal position of the CG, which serves as the moment reference point.
% This is often placed near the quarter-chord of the Mean Aerodynamic Chord.
% We will calculate this automatically later, but you can override it.
% x_cg_override = 2.5; % Uncomment to manually set the CG

% --- Output Filename ---
datcom_filename = 'datcom_example.dat';

% =========================================================================
% SECTION 2: CALCULATE WING VERTEX COORDINATES FOR PLOTTING
% =========================================================================
% Convert angles to radians for MATLAB's trig functions
sweep_rad = deg2rad(sweep_deg);
dihedral_rad = deg2rad(dihedral_deg);

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
right_wing_vertices = [v1; v2; v4; v3];
right_wing_faces = [1, 2, 3, 4];

% Create the left wing by negating the Y-coordinates
%left_wing_vertices = right_wing_vertices;
%left_wing_vertices(:, 2) = -left_wing_vertices(:, 2);


% =========================================================================
% SECTION 3: PLOT THE 3D WING
% =========================================================================
disp('Generating 3D plot of the wing...');
figure('Name', '3D Wing Geometry');
hold on;

% Plot the right wing
patch('Vertices', right_wing_vertices, 'Faces', right_wing_faces, ...
      'FaceColor', [0.5, 0.7, 0.9], 'EdgeColor', 'k', 'LineWidth', 1.5);

% Plot the left wing
%patch('Vertices', left_wing_vertices, 'Faces', right_wing_faces, ...
%      'FaceColor', [0.5, 0.7, 0.9], 'EdgeColor', 'k', 'LineWidth', 1.5);

% Formatting the plot
title('Generated Wing Geometry');
xlabel('X - Longitudinal Axis (ft)');
ylabel('Y - Spanwise Axis (ft)');
zlabel('Z - Vertical Axis (ft)');
grid off;
axis equal; % Ensures the aspect ratio is realistic
view(30, 25); % Set a nice 3D viewing angle
hold off;


% =========================================================================
% SECTION 4: CALCULATE DATCOM REFERENCE PARAMETERS FOR A SINGLE WING
% =========================================================================
% Wing span (for a single panel, this is just the semi-span)
blref = semi_span;

% Taper ratio (lambda)
taper_ratio = tip_chord / root_chord;

% Wing area (area of one trapezoid)
sref = semi_span * (root_chord + tip_chord) / 2;

% Mean Aerodynamic Chord (MAC)
cbarr = (2/3) * root_chord * (1 + taper_ratio + taper_ratio^2) / (1 + taper_ratio);

% Longitudinal position of the CG (moment reference point)
if exist('x_cg_override', 'var')
    xcg = x_cg_override;
else
    % Default: Place CG at the quarter-chord point of the MAC
    % Note: This is a simplified calculation for x_mac_le for a tapered, swept wing.
    y_mac = (semi_span/3) * ( (1 + 2*taper_ratio)/(1+taper_ratio) );
    x_mac_le = wing_apex_x + y_mac * tan(sweep_rad);
    xcg = x_mac_le + 0.25 * cbarr;
end

% =========================================================================
% SECTION 5: WRITE THE DATCOM FILE
% =========================================================================
% Total wing span (now just the semi-span for one wing)
blref = semi_span;

% Taper ratio (lambda)
taper_ratio = tip_chord / root_chord;

% Total wing area (area of one trapezoid)
sref = (semi_span * (root_chord + tip_chord) / 2);

% Mean Aerodynamic Chord (MAC)
cbarr = (2/3) * root_chord * (1 + taper_ratio + taper_ratio^2) / (1 + taper_ratio);

% Longitudinal position of the CG (moment reference point)
if exist('x_cg_override', 'var')
    xcg = x_cg_override;
else
    % Default: Place CG at the quarter-chord point of the MAC
    % Note: This is a simplified calculation for x_mac_le for a tapered, swept wing.
    y_mac = (semi_span/3) * ( (1 + 2*taper_ratio)/(1+taper_ratio) );
    x_mac_le = wing_apex_x + y_mac * tan(sweep_rad);
    xcg = x_mac_le + 0.25 * cbarr;
end


% =========================================================================
% SECTION 5: WRITE THE DATCOM FILE (Corrected Formatting)
% =========================================================================
disp(['Writing DATCOM file: ', datcom_filename, '...']);

% Open the file for writing (deletes previous content)
fileID = fopen(datcom_filename, 'w');
if fileID == -1
    error('Could not open file for writing.');
end

% --- Write Header and Commands ---
fprintf(fileID, 'DIM M\n');
%fprintf(fileID, 'DAMP\n');
fprintf(fileID, 'DERIV DEG\n');

% --- Helper function to format arrays with commas ---
format_array = @(arr) strjoin(arrayfun(@(x) sprintf('%.4f', x), arr, 'UniformOutput', false), ',');

% --- Write $FLTCON Namelist ---
%fprintf(fileID, ' $FLTCON NMACH=%.1f, MACH(1)=%s,\n', length(mach_numbers), format_array(mach_numbers));
%fprintf(fileID, ' NALT=%.1f, ALT(1)=%s,\n', length(altitudes_ft), format_array(altitudes_ft));
%fprintf(fileID, ' NALPHA=%.1f, ALSCHD(1)=%s,\n', length(alphas_deg), format_array(alphas_deg));
%fprintf(fileID, ' LOOP=2.0$\n');

if ~isempty(mach_numbers)
    fprintf(fileID, ' $FLTCON NMACH=%.1f, MACH(1)=%.4f,\n', length(mach_numbers), mach_numbers(1));
    for i = 2:length(mach_numbers)
        fprintf(fileID, ' %.4f,\n', mach_numbers(i)); % Indent values for clarity
    end
end

% --- Write subsequent variables ---
% Note: These do NOT start with '$FLTCON'
if ~isempty(altitudes_ft)
    fprintf(fileID, ' NALT=%.1f, ALT(1)=%.4f,\n', length(altitudes_ft), altitudes_ft(1));
    for i = 2:length(altitudes_ft)
        fprintf(fileID, ' %.4f,\n', altitudes_ft(i));
    end
end

if ~isempty(alphas_deg)
    fprintf(fileID, ' NALPHA=%.1f, ALSCHD(1)=%.4f,\n', length(alphas_deg), alphas_deg(1));
    % Loop to the SECOND-TO-LAST element, printing a comma
    for i = 2:length(alphas_deg)
        fprintf(fileID, ' %.4f,\n', alphas_deg(i));
    end
end
fprintf(fileID, ' LOOP=2.0$\n');
% --- Write $OPTINS Namelist ---
fprintf(fileID, ' $OPTINS SREF=%.4f, CBARR=%.4f, BLREF=%.4f$\n', sref, cbarr, blref);

% --- Write $SYNTHS Namelist ---
fprintf(fileID, ' $SYNTHS XW=%.4f, XCG=%.4f$\n', wing_apex_x, xcg);

% --- Write $WGPLNF Namelist ---
fprintf(fileID, ' $WGPLNF CHRDR=%.4f, CHRDTP=%.4f, SSPN=%.4f, SSPNE=%.4f,\n', ...
    root_chord, tip_chord, semi_span, semi_span);
fprintf(fileID, ' SAVSI=%.4f, DHDADI=%.4f, TYPE=1.0$\n', sweep_deg, dihedral_deg);

% --- Write NACA Airfoil Card ---
% Parse the airfoil string to fit DATCOM format
parts = strsplit(airfoil_naca);
if length(parts) == 2 && strcmpi(parts{1}, 'NACA') && (length(parts{2}) == 4 || length(parts{2}) == 5)
    if length(parts{2}) == 4
        naca_series = '4';
    else
        naca_series = '5';
    end
    naca_profile = parts{2};
    fprintf(fileID, 'NACA W %s %s\n', naca_series, naca_profile);
else
    warning('Airfoil format not recognized. Writing a default NACA 0012.');
    fprintf(fileID, 'NACA W 4 0012\n');
end

% --- Write Case Control ---
%fprintf(fileID, 'NACA W %s %s\n', naca_series, naca_profile);

% --- Write Case Control (NO LEADING SPACE) ---
%fprintf(fileID, 'CASEID GENERATED WING-ONLY ANALYSIS\n');
fprintf(fileID, 'NEXT CASE\n'); % Column 1

% --- Close the file ---
fclose(fileID);

disp('DATCOM file successfully generated!');

% 1. Setup File and Executable Names
%input_filename = 'wing_model.dat';

% The 'system' command executes the program. It will pause MATLAB until
% DATCOM is finished. A status of 0 means it ran without OS-level errors.
disp('Running DATCOM... (MATLAB will pause)');

if ispc
    % --- Windows ---
    executable_name = 'datcom_win.exe';
    if exist(executable_name, 'file') ~= 2
        error('"%s" not found in the current directory.', executable_name);
    end
    command = sprintf('echo %s | %s', datcom_filename, executable_name);
elseif isunix || ismac
    % --- Linux / macOS ---
    executable_name = './datcom_unix';
    
    % 1. Force the input file to be 'FOR005'
    % Many Linux builds are hardcoded to unit 5 -> FOR005
    copyfile(datcom_filename, 'FOR005');
    
    % 2. Pipe the filename string INTO the program
    % We use \n to simulate hitting 'Enter'
    command = sprintf('echo "FOR005\n" | %s', executable_name);
    
    [status, cmdout] = system(command);
    
    % 3. Cleanup: DATCOM loves to leave mess
    if exist('FOR006', 'file'), movefile('FOR006', 'datcom.out'); end
else
    error('Unsupported operating system.');
end
status = system(command);

if status ~= 0
    fprintf('DATCOM Error Output:\n%s\n', cmdout);
    error('DATCOM failed to execute correctly.');
end


disp('DATCOM execution finished.');

aero_cell = datcomimport('datcom.out');
aero = aero_cell{1};

disp('Generating aerodynamic plots...');
plot_altitude_index = 2;

% --- Validate Plotting Index ---
if plot_altitude_index > length(aero.alt) || plot_altitude_index < 1
    error('Invalid plot_altitude_index. It must be between 1 and %d.', length(aero.alt));
end

figure('Name', 'Aerodynamic Coefficients');
hold on;
grid on;

% --- Loop Through Each Mach Number to Plot CL vs Alpha ---
legend_entries = cell(1, length(aero.mach));
for i = 1:length(aero.mach)
    % Extract the CL data for the current Mach number and the chosen altitude
    % The dimensions of aero.cl are (alpha, mach, altitude)
    current_cd = aero.cd(:, i, plot_altitude_index);

    % Plot the data
    plot(aero.alpha, current_cd, 'LineWidth', 2, 'Marker', 'o', 'MarkerSize', 4);
    
    % Store the legend entry for this line
    legend_entries{i} = sprintf('Mach = %.2f', aero.mach(i));
end

% --- Format the Plot ---
title(sprintf('Lift Coefficient vs. Angle of Attack at %.0f ft', aero.alt(plot_altitude_index)));
xlabel('Angle of Attack, \alpha (degrees)');
ylabel('Drag Coefficient, C_D');
legend(legend_entries, 'Location', 'SouthEast');
hold off;

