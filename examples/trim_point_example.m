close all; clear; clc;

% Define the file path to the flight data CSV file
file_path = 'Flight Profile\flight_2.csv';

% Create a TrimPoint object by initializing it with the specified flight data file
trim_point = TrimPoint(file_path);

% Find the time of the "LIFTOFF" event in the flight data
liftoff_time = trim_point.find_event_time("LIFTOFF");

% Find the time of the "BURNOUT" event in the flight data
burnout_time = trim_point.find_event_time("BURNOUT");

% Number of batches of trim points to add
num_batches = 10;

% Generate an array of 6 evenly spaced time points between liftoff_time and burnout_time
time_arr = linspace(liftoff_time + 0.2, burnout_time, num_batches);

% Generate trim batches for each time point in time_arr
% Each batch corresponds to a specific time instance and contains variations based on the base_state
trim_point.generate_trim_batches(time_arr);

% Define the number of trim points to generate per time instance for 'u' variations
num_trim_points_per_time = 12;

% Define the spacing factor for 'u' variations (e.g., 20% variation around the base 'u' value)
trim_spacing = 0.18;

% Add 'u' (velocity) variations to each trim batch based on the specified parameters
% This creates different trim points by varying the 'u' component within the defined spacing
trim_point.add_u_variations(time_arr, num_trim_points_per_time, trim_spacing);

turn_batch_index = 3:num_batches;  % Define the indices of batches to which custom variations will be added
turn_variation_name = "turn";  % Define the name of the custom variation to add

% Define the variables involved in the custom variation
% In this case, the variation affects the quaternion components q0, q1, q2, and q3
turn_varables = ["q0", "q1", "q2", "q3"];

% Define the variation data matrix for the custom variation
% Each row represents a different state for the variation
turn_data = [0.0436194, 0, 0, 0.9990482;   % 5 deg
             0.0871557, 0, 0, 0.9961947;   % 10 deg
             0.1305262, 0, 0, 0.9914449];  % 15 deg

% include base state [0, 0, 0, 1] strait up flight
turn_include_base = true;

% Add the custom "turn" variation to the specified batches (2, 3, and 4)
% This will create additional trim points by varying the specified quaternion components
trim_point.add_custom_variations(turn_batch_index, turn_variation_name, turn_varables, turn_data, turn_include_base);

yaw_batch_index = 4:num_batches;
yaw_variation_name = "yaw";
yaw_varables = "Yaw";
yaw_data = [40; 80];
include_base = true;

trim_point.add_custom_variations(yaw_batch_index, yaw_variation_name, yaw_varables, yaw_data, include_base)

% Generate all trim points by combining variations from all batches
% This consolidates all trim points into the obj.trim_points property for further use
trim_point.generate_trim_points();

% Get trim points closest to 1 second
one_s_trim_ponts = trim_point.get_trim_points(1);

% Export csv
trim_point.export_trim_points_to_csv('examples/trim_points.csv');

% % Visualize 'u' variations (assuming a draw_u_vars method exists)
trim_point.draw_u_trim_points(0, burnout_time)