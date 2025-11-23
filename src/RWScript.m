
jsonFileName = 'RWParams.json'; % Json with DATCOM Parameters
jsonString = fileread(jsonFileName);
jsonData = jsondecode(jsonString);

% Parameters that need to double checked
% xRef
% All flight parameters

controller = -1;
environment = -1;
breakPoints = -1;

rocketDyn = setupRocket(controller, jsonData)
runDATCOM(rocketDyn);
flight = setupFlight(rocketDyn, environment, breakPoints, jsonData)



function rocketDyn = setupRocket(controller, jsonData)

    % Extract parameters for rocket setup
    rocketParams = jsonData.rocket;
    bodyParams = jsonData.body;
    finParams = jsonData.fins;
    motorParams = jsonData.motor;
    flightConParams = jsonData.flightCon;
    
    % Calculate total rocket length
    totalLength = bodyParams.noseLength + bodyParams.bodyLength;
    bodyParams.cgXWet = rocketParams.cgXWet;
    
    % Calculate fin x position
    xFinRef = totalLength - finParams.rootChord;
    finParams.translation = [xFinRef, bodyParams.rocketDiameter / 2, 0];
    
    % Create instances of RocketBody and RocketFins using the extracted parameters
    body = rocketPkg.RocketBody(bodyParams, flightConParams);
    fins = rocketPkg.RocketFins(finParams, flightConParams);
    
    aerodynamics = [];
    
    % Initialize the rocket dynamics simulation parameters
    rocketDyn = rocketPkg.RocketDynamics(body, fins, rocketParams, motorParams, controller, aerodynamics);

end

% Run DATCOM to compute aerodynamics
function runDATCOM(rocketDyn)

    rocketDyn.computeBuildup();

end

% Setup flight object
function flight = setupFlight(rocketDyn, breakPoints, environment, jsonData)

    flightParams = jsonData.flight;
    flightParams.x0 = [rocketDyn.massWet, jsonData.flight.pos0', jsonData.flight.vel0', jsonData.flight.quat0', jsonData.flight.omega0'];
    nav = -1;
    
    flight = flightPkg.Flight(rocketDyn, environment, breakPoints, nav, flightParams);

end

