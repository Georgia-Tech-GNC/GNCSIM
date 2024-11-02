close all; clear; clc;

file_path = 'Flight Profile\flight_2.csv';

trim_point = TrimPoint(file_path);

liftoff_time = trim_point.find_event_time("LIFTOFF");
burnout_time = trim_point.find_event_time("BURNOUT");

time_arr = linspace(liftoff_time, burnout_time, 6);

base_state = trim_point.get_vertical_state();


trim_point.generate_trim_batches(time_arr)

num_trim_points_per_time = 15;
trim_spacing = .2;

trim_point.add_u_variations(time_arr, num_trim_points_per_time, trim_spacing)

batch_index = [2,3,4];
var_name = "turn";
varables = ["q0", "q1", "q2", "q3"];
data = [0.9962, 0, 0, 0.0872;
        1, 0, 0, 0];

trim_point.add_custom_variations(batch_index, var_name, varables, data);
trim_point.generate_trim_points()


%trim_point.generate_trim_points_from_u_var(time_arr, num_trim_points_per_time, trim_spacing)

%trim_point.draw_u_vars()

