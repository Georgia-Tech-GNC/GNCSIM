close all; clear; clc;

changed()

file_path = 'Flight Profile\flight_2.csv';
time_arr = linspace(0, 2, 5);
points = 10;

trim_point = TrimPoint(file_path, time_arr, points);



function changed()
    currentFilePath = mfilename('fullpath');
    [currentFileDir, ~, ~] = fileparts(currentFilePath);
    parentDir = fileparts(currentFileDir);
    cd(parentDir);
end


