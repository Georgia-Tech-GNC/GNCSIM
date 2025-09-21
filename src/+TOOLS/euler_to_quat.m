function q = euler_to_quat(a)
    % Convert Euler angles to a quaternion.
    %
    % Input:
    %   a: A 1x3 or 3x1 vector of Euler angles [roll, pitch, yaw] in degrees.
    %
    % Output:
    %   q: A 4x1 vector representing the quaternion [w, x, y, z].

    % Convert the angles from degrees to radians
    a = deg2rad(a);

    % Pre-calculate sines and cosines of half angles
    cy = cos(a(2) * 0.5);  % Cosine of half pitch
    sy = sin(a(2) * 0.5);  % Sine of half pitch
    cr = cos(a(1) * 0.5);  % Cosine of half roll
    sr = sin(a(1) * 0.5);  % Sine of half roll
    cp = cos(a(3) * 0.5);  % Cosine of half yaw
    sp = sin(a(3) * 0.5);  % Sine of half yaw

    % Pre-allocate the output quaternion as a column vector
    q = zeros(4, 1);

    % Calculate the quaternion components
    q(1) = cy * cr * cp + sy * sr * sp;  % w component
    q(2) = cy * sr * cp - sy * cr * sp;  % x component
    q(3) = sy * cr * cp - cy * sr * sp;  % y component
    q(4) = cy * cr * sp + sy * sr * cp;  % z component
end