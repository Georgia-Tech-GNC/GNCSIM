
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


function rocketDyn = setupRocket(jsonData)

% Extract parameters for rocket setup
rocketParams = jsonData.rocket;
bodyParams = jsonData.body;
finParams = jsonData.fins;
motorParams = jsonData.motor;
flightConParams = jsonData.flightCon;

% Calculate total rocket length
totalLength = bodyParams.nose_cone_length + bodyParams.body_length;

% Calculate fin x position
xFinRef = totalLength - rocketParams.cg_x - finParams.root_chord;
finParams.translation = [xFinRef, bodyParams.rocket_diameter / 2, 0];

% Create instances of RocketBody and RocketFins using the extracted parameters
body = rocket_pkg.RocketBody(bodyParams, flightConParams);
fins = rocket_pkg.RocketFins(finParams, flightConParams);

% Initialize the rocket dynamics simulation parameters
rocketDyn = rocket_pkg.RocketDynamics(body, fins, rocketParams, motorParams);

end

function runDATCOM(rocketDyn)

rocketDyn.compute_buildup();

end

function flight = setupFlight(rocketDyn)

end

%rocket_dyn = setupRocket(jsonData)
%runDATCOM(rocket_dyn);
