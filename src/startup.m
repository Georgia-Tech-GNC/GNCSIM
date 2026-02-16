%% GNCSIM Project Startup Script
% This script runs automatically when the project opens.
% It verifies that all required toolboxes are installed.

disp("Initializing GNCSIM project environment...")

%% Required Toolboxes
requiredToolboxes = {
    'Aerospace Toolbox'
    'Control System Toolbox'
    'MATLAB'
    'MATLAB Report Generator'
    'Mapping Toolbox'
    'Navigation Toolbox'
    'Signal Processing Toolbox'
    'Symbolic Math Toolbox'
};

%% Get installed toolboxes
v = ver;
installedToolboxes = {v.Name};

%% Check for missing toolboxes
missing = requiredToolboxes(~ismember(requiredToolboxes, installedToolboxes));

if ~isempty(missing)
    fprintf(2, '\nERROR: Missing required toolboxes:\n')
    for i = 1:numel(missing)
        fprintf(2, '  - %s\n', missing{i});
    end
    error('Project initialization failed due to missing toolboxes.')
else
    disp("All required toolboxes detected.")
end

disp("GNCSIM environment ready.")
