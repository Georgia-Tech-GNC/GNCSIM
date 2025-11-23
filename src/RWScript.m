
jsonFileName = 'RWParams.json'; % Json with DATCOM Parameters
jsonString = fileread(jsonFileName);
jsonData = jsondecode(jsonString);

% Parameters that need to double checked
% moi_wet
% mot_dry
% CG_wet
% CG_dry
% x_ref
% All flight parameters


function rocket_dyn = setupRocket(jsonData)

% Extract parameters for rocket setup
rocket_params = jsonData.rocket;
body_params = jsonData.body;
fin_params = jsonData.fins;
motor_params = jsonData.motor;
flight_con_params = jsonData.flight;

% Calculate total rocket length
total_length = body_params.nose_cone_length + body_params.body_length;

% Calculate fin x position
x_fin_ref = total_length - rocket_params.cg_x - fin_params.root_chord;
fin_params.translation = [x_fin_ref, body_params.rocket_diameter / 2, 0];

% Create instances of RocketBody and RocketFins using the extracted parameters
body = rocket_pkg.RocketBody(body_params, flight_con_params);
fins = rocket_pkg.RocketFins(fin_params, flight_con_params);

% Initialize the rocket dynamics simulation parameters
rocket_dyn = rocket_pkg.RocketDynamics(body, fins, rocket_params, motor_params);

end

% rocket_dyn = setupRocket(jsonData)