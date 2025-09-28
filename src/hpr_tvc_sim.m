% moment simulation for HPR TVC
clear; clc;

% Run simulation
dt = 0.01;
t_final = 3;
steps = t_final / dt;
x0 = [1; 0; 0; 0; 0.0; 0.0; 0.0];  % [q0 q1 q2 q3 wx wy wz]
[x_arr, t_arr] = rk4(@dynamics, x0, 0, dt, steps);

% Plot results
figure;
subplot(3,1,1);
plot(t_arr, x_arr(1:4,:));
legend('q0', 'q1', 'q2', 'q3');
title('Quaternions');

subplot(3,1,2);
plot(t_arr, x_arr(5:7,:));
legend('wx', 'wy', 'wz');
title('Angular Velocities (rad/s)');

% Convert quaternions to Euler angles and plot
% Replace your Euler angle conversion section with:
euler_angles = zeros(3, length(t_arr));
for i = 1:length(t_arr)
    quat = x_arr(1:4, i)';
    euler = quat2eul(quat, 'ZYX');
    
    % Unwrap angles to remove discontinuities
    if i > 1
        euler = unwrap([euler_angles(:, i-1)'; euler]);
        euler = euler(2, :);
    end
    
    euler_angles(:, i) = euler';
end

subplot(3,1,3);
plot(t_arr, rad2deg(euler_angles));
legend('Yaw (Z)', 'Pitch (Y)', 'Roll (X)');
title('Euler Angles (degrees)');
xlabel('Time (s)');

function x_dot = dynamics(x)
	% Inertia matrix (kg*m^2) - scaled for larger rocket
	Ib = diag([0.2, 0.2, 0.0036]);

	M_a = [0; -2.755; 0]; %Nm

    T = 35;  %N
    max_deflection = 17; %deg

    thrust_moment = T .* sin(deg2rad(max_deflection)) .* 0.291846

	M_T = [0; thrust_moment; 0]; %Nm
	M_b = M_a + M_T;

	q0 = x(1);
	q1 = x(2);
	q2 = x(3);
	q3 = x(4);

	wx = x(5);
	wy = x(6);
	wz = x(7);
	wbf_b = [wx; wy; wz];
	
	wbf_b_dot = Ib\(M_b - cross(wbf_b, Ib*wbf_b));

	% Normalize quaternion to prevent drift
	quat_norm = sqrt(q0^2 + q1^2 + q2^2 + q3^2);
	q0_norm = q0 / quat_norm;
	q1_norm = q1 / quat_norm;
	q2_norm = q2 / quat_norm;
	q3_norm = q3 / quat_norm;

	% Recalculate derivatives with normalized quaternions
	q0_dot = 0.5 * (-wx*q1_norm - wy*q2_norm - wz*q3_norm);
	q1_dot = 0.5 * (wx*q0_norm + wz*q2_norm - wy*q3_norm);
	q2_dot = 0.5 * (wy*q0_norm - wz*q1_norm + wx*q3_norm);
	q3_dot = 0.5 * (wz*q0_norm + wy*q1_norm - wx*q2_norm);
	q_bf_dot = [q0_dot;q1_dot;q2_dot;q3_dot];

	x_dot = [q_bf_dot; wbf_b_dot];
end

function [x_arr, t_arr] = rk4(f, x0, t0, dt, steps)
	x_arr = zeros(length(x0), steps+1);
	t_arr = zeros(1, steps+1);
	x_arr(:, 1) = x0;
	t_arr(1) = t0;

	for i = 1:steps
	    t = t_arr(i);
	    x = x_arr(:, i);
	    
	    k1 = dt * f(x);
	    k2 = dt * f(x + k1/2);
	    k3 = dt * f(x + k2/2);
	    k4 = dt * f(x + k3);
	    
	    x_arr(:, i+1) = x + (k1 + 2*k2 + 2*k3 + k4)/6;
	    t_arr(i+1) = t + dt;

        % Normalize quaternion after each step
        quat_norm = norm(x_arr(1:4, i+1));
        x_arr(1:4, i+1) = x_arr(1:4, i+1) / quat_norm;
	end
	
end
