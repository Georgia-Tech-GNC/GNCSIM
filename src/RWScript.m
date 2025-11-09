
jsonFileName = 'RWParams.json';
jsonString = fileread(jsonFileName);
jsonData = jsondecode(jsonString);

function rocket_dyn = setupRocket(jsonData)

rocket_params = jsonData.rocket;
body_params = jsonData.body;
fin_params = jsonData.fins;
motor_params = jsonData.motor;
flight_con_params = jsonData.flight;

total_length = body_params.nose_cone_length + body_params.body_length;
x_fin_ref = total_length - rocket_params.cg_x - fin_params.root_chord;

fin_params.translation = [x_fin_ref, body_params.rocket_diameter / 2, 0];

body = rocket_pkg.RocketBody(body_params, flight_con_params);
fins = rocket_pkg.RocketFins(fin_params, flight_con_params);

rocket_dyn = rocket_pkg.RocketDynamics(body, fins, rocket_params, motor_params);
end

rocket_dyn = setupRocket(jsonData);