clear;
clc;
close all;

accelerometer = Sensor();
gyro = Gyro(obj.gyroBias, obj.gyroRange, obj.gyroNoise_density, obj.gyroSampling_rate);
gps;
dt = [];
x0 = [0;0;0;0;0;0];
P0 = 0.01 .* eye(6);
sigma_a = 10;
q0 = [1;0;0;0];
Q = sigma_a^2 .* [(dt^4)/4, (dt^3)/2,                    0,                    0,                    0,                    0;
                           (dt^3)/2,     dt^2,                    0,                    0,                    0,                    0;
                                              0,                    0, (dt^4)/4, (dt^3)/2,                    0,                    0;
                                              0,                    0, (dt^3)/2,     dt^2,                    0,                    0;
                                              0,                    0,                    0,                    0, (dt^4)/4, (dt^3)/2;
                                              0,                    0,                    0,                    0, (dt^3)/2,    dt^2];

R = [4.2 0 0;
              0 8.4 0;
              0 0 8.4];

f = @(x,dt,a_b) state_transition_fn(x,dt,a_b);
h = @(x)[x(1);x(2);x(3)];
dfdx = @(dt, q)[1, dt*(2*q(1)^2 + 2*q(2)^2 - 1), 0,   dt*(2*q(1)*q(4) + 2*q(2)*q(3)), 0,  -dt*(2*q(1)*q(3) - 2*q(2)*q(4))
            0,                        1, 0,                        0, 0,                        0
            0,  -dt*(2*q(1)*q(4) - 2*q(2)*q(3)), 1, dt*(2*q(1)^2 + 2*q(3)^2 - 1), 0,   dt*(2*q(1)*q(2) + 2*q(3)*q(4))
            0,                        0, 0,                        1, 0,                        0
            0,   dt*(2*q(1)*q(3) + 2*q(2)*q(4)), 0,  -dt*(2*q(1)*q(2) - 2*q(3)*q(4)), 1, dt*(2*q(1)^2 + 2*q(4)^2 - 1)
            0,                        0, 0,                        0, 0,                        1];
dhdx = [1 0 0 0 0 0;
            0 0 1 0 0 0;
            0 0 0 0 1 0];

navObj = Navigation(x, P0, q0, Q, R, f, dfdx, h, dhdx, accelerometer, gyro, gps);
%%
load('data/6dof_traj.mat') % Generates struct of flight simulation data named "log"

%%
ts = log.loggedData.time;
for i = 1:length(ts)-1

    t = ts(i);
    dt = ts(i+1) - ts(i);
    navObj.readSensors(log, t);
    navObj.runAttitudeEstimation(dt);
    navObj.runEKF(dt);

end

%% What we need to convert from old code

% function f = state_transition_fn(ekf,atdf,sensor)
%     % Calculates next state for translational EKF
%     a_b = sensor.accelerometer; % this is true acceleration    velFlat_n = quat_rotate(quatconj(atdf.q_current'), [ekf.x_n(2) ekf.x_n(4) ekf.x_n(6)]);    f1 = ekf.x_n(1) + velFlat_n(1)*ekf.dt;
%     f2 = ekf.x_n(2) + ekf.dt*a_b(1);
%     f3 = ekf.x_n(3) + velFlat_n(2)*ekf.dt;
%     f4 = ekf.x_n(4) + ekf.dt*a_b(2);
%     f5 = ekf.x_n(5) + velFlat_n(3)*ekf.dt;
%     f6 = ekf.x_n(6) + ekf.dt*a_b(3);    f = [f1;f2;f3;f4;f5;f6];
% endfunction dfdx = state_transition_jacob(ekf,atdf)
%     % Calculates state transition jacobian for translational EKF    dt = ekf.dt;    q_fb = quatconj(atdf.q_current');    q0 = q_fb(1);
%     q1 = q_fb(2);
%     q2 = q_fb(3);
%     q3 = q_fb(4);    dfdx = [1, dt*(2*q0^2 + 2*q1^2 - 1), 0,   dt*(2*q0*q3 + 2*q1*q2), 0,  -dt*(2*q0*q2 - 2*q1*q3)
%             0,                        1, 0,                        0, 0,                        0
%             0,  -dt*(2*q0*q3 - 2*q1*q2), 1, dt*(2*q0^2 + 2*q2^2 - 1), 0,   dt*(2*q0*q1 + 2*q2*q3)
%             0,                        0, 0,                        1, 0,                        0
%             0,   dt*(2*q0*q2 + 2*q1*q3), 0,  -dt*(2*q0*q1 - 2*q2*q3), 1, dt*(2*q0^2 + 2*q3^2 - 1)
%             0,                        0, 0,                        0, 0,                        1];endfunction h = observation_fn(ekf)
%     % Calculates state-to-measurement relation for translational EKF    h1 = ekf.x_n(1);
%     h2 = ekf.x_n(3);
%     h3 = ekf.x_n(5);
% %     h4 = ekf.x_n(4);
% %     h5 = ekf.x_n(5);
% %     h6 = ekf.x_n(6);    h = [h1;h2;h3];endfunction dhdx = observation_jacob(ekf)
%     % Calculates observation jacobian for translational EKF    dhdx = [1 0 0 0 0 0;
%             0 0 1 0 0 0;
%             0 0 0 0 1 0];end


function [x_tplus1] = state_transition_fn(x_n, dt, accel_body)
    velFlat_n = quat_rotate(quatconj(atdf.q_current'), [x_n(2) x_n(4) x_n(6)]);
    f1 = x_n(1) + velFlat_n(1)*dt;
    f2 = x_n(2) + dt*accel_body(1);
    f3 = x_n(3) + velFlat_n(2)*dt;
    f4 = x_n(4) + dt*accel_body(2);
    f5 = x_n(5) + velFlat_n(3)*dt;
    f6 = x_n(6) + dt*accel_body(3);   
    x_tplus1 = [f1;f2;f3;f4;f5;f6];

end
