paramsPath = fullfile(pwd, "+SyndromeFiles", "params.json");
params = jsondecode(fileread(paramsPath));

if ~exist('params','var') && exist('paramsPath','var')
    S = load(paramsPath);
    if isfield(S,'params')
        params = S.params;
    else
        % collect all fields into params
        fn = fieldnames(S);
        params = struct();
        for ii=1:numel(fn), params.(fn{ii}) = S.(fn{ii}); end
    end
end
if ~exist('params','var')
    params = struct(); % empty defaults
end

% Provide safe defaults for missing sub-structs
if ~isfield(params,'rocketBody'), params.rocketBody = struct(); end
if ~isfield(params,'rocketFins'), params.rocketFins = struct(); end
if ~isfield(params,'rocketDynamics'), params.rocketDynamics = struct(); end
if ~isfield(params,'environment'), params.environment = struct(); end
if ~isfield(params,'flightParams'), params.flightParams = struct(); end
if ~isfield(params,'log'), params.log = struct(); end
if ~isfield(params,'controller'), params.controller = struct(); end
if ~isfield(params,'navigation'), params.navigation = struct(); end
if ~isfield(params,'flight'), params.flight = struct(); end

% --- Construct subsystems when classes available; otherwise create placeholders ---
% RocketBody
try
    rocketBody = RocketBody(params.rocketBody);
catch
    rocketBody = params.rocketBody;
    warning('Syndrome:RocketBodyMissing','RocketBody class not found. Placeholder struct used.');
end

% RocketFins
try
    rocketFins = RocketFins(params.rocketFins);
catch
    rocketFins = params.rocketFins;
    warning('Syndrome:RocketFinsMissing','RocketFins class not found. Placeholder struct used.');
end

% RocketDynamics
try
    rocketDynamics = RocketDynamics(params.rocketDynamics);
catch
    rocketDynamics = params.rocketDynamics;
    warning('Syndrome:RocketDynamicsMissing','RocketDynamics class not found. Placeholder struct used.');
end

% Environment
try
    environment = Environment(params.environment);
catch
    environment = params.environment;
    warning('Syndrome:EnvironmentMissing','Environment class not found. Placeholder struct used.');
end

% Flight params and log
flightParams = params.flightParams;
logObj = params.log;

% Controller
try
    controller = Controller(params.controller);
catch
    controller = params.controller;
    warning('Syndrome:ControllerMissing','Controller class not found. Placeholder struct used.');
end

% Navigation (allow either instance or params)
try
    if isobject(params.navigation)
        navigation = params.navigation;
    elseif isfield(params.navigation,'class') && ischar(params.navigation.class)
        cls = params.navigation.class;
        if isfield(params.navigation,'params')
            navigation = feval(cls, params.navigation.params);
        else
            navigation = feval(cls, params.navigation);
        end
    else
        % default to navigationPkg.SimpleNavigation if available
        navigation = navigationPkg.SimpleNavigation(params.navigation);
    end
catch
    navigation = params.navigation;
    warning('Syndrome:NavigationMissing','Navigation class not found. Placeholder used.');
end

% Flight (top-level)
try
    flight = Flight(rocketBody, rocketDynamics, rocketFins, environment, controller, navigation, flightParams);
catch
    try
        flight = Flight(params.flight);
    catch
        flight = params.flight;
        warning('Syndrome:FlightMissing','Flight class not found. Placeholder used.');
    end
end

% Actuator state names: prefer user-provided, otherwise default
if isfield(params,'actuatorStateNames') && iscellstr(params.actuatorStateNames)
    actuatorStateNames = params.actuatorStateNames;
else
    actuatorStateNames = {'tau_applied','omega_w'}; % default; change to match actuator model
end

% Final brief report
fprintf('Syndrome setup complete.\n');
fprintf('  rocketBody:        %s\n', classOrType(rocketBody));
fprintf('  rocketFins:        %s\n', classOrType(rocketFins));
fprintf('  rocketDynamics:    %s\n', classOrType(rocketDynamics));
fprintf('  environment:       %s\n', classOrType(environment));
fprintf('  controller:        %s\n', classOrType(controller));
fprintf('  navigation:        %s\n', classOrType(navigation));
fprintf('  flight:            %s\n', classOrType(flight));
fprintf('  actuatorStateNames: %s\n', strjoin(actuatorStateNames, ', '));

% Helper nested function for printing type
function s = classOrType(x)
    if isempty(x)
        s = '<empty>';
    elseif isobject(x)
        s = class(x);
    elseif isstruct(x)
        s = '<struct>';
    else
        s = class(x);
    end
end