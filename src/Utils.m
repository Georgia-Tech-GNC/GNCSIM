classdef Utils
    methods (Static)
        % get rotation matrix from a given quaternion
        function vb = quatRotate(q,va)
            % quat_rotate Rotates vec data set using qab from b to a i.e.
            % qBI is Inertial to Body
            % va - 3d vector in b frame (1x3 row or 3x1 column)
            % vb - 3d vec in a frame (same shape as va)

            % Check if va is column vector and convert to row
            was_column = false;
            if size(va, 1) == 3 && size(va, 2) == 1
                va = va';
                was_column = true;
            end

            % Normalize quaternion
            qU = Utils.normalizeQuat(q);

            % quat rotation: q_conj * [0, va] * q
            % Convert vector to pure quaternion [0, vx, vy, vz]
            vaQuat = [0, va];
            [~, vB1, vB2, vB3] = parts(quaternion(quatconj(qU)) .* quaternion(vaQuat) .* quaternion(qU));
            vb = [vB1, vB2, vB3];

            % Convert back to column if input was column
            if was_column
                vb = vb';
            end
        end

        function q = eulerToQuat(a)
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

        function qNorm = normalizeQuat(q)
            % normalize_quat: Normalizes a quaternion to unit length.
            %
            % Input:
            %   q: A quaternion as a 4-element vector [w, x, y, z] (row or column).
            %      Can also be an Nx4 matrix where each row is a quaternion.
            %
            % Output:
            %   q_norm: The normalized quaternion(s) with the same shape as input.

            % Ensure q is treated correctly whether it's a row, column, or matrix
            original_shape = size(q);

            % If it's a column vector (4x1), transpose to row for consistency
            if original_shape(1) == 4 && original_shape(2) == 1
                q = q';
                was_column = true;
            else
                was_column = false;
            end

            % Compute the magnitude of each quaternion
            mag = sqrt(sum(q.^2, 2));

            % Normalize
            qNorm = q ./ mag;

            % Restore original shape if it was a column vector
            if was_column
                qNorm = qNorm';
            end
        end

        function dcm = dirCosine(q)
            % dirCosine: Converts a quaternion to a direction cosine matrix.
            %
            % This function returns a 3x3 matrix that represents a rotation
            % from an INERTIAL frame to a BODY frame, based on the input quaternion.
            %
            % INPUT:
            %   q - A 4-element column or row vector representing the quaternion [q0; q1; q2; q3],
            %       where q0 is the scalar part.
            %
            % OUTPUT:
            %   dcm - A 3x3 direction cosine matrix (rotation matrix).
            %

            % Ensure q is a column vector
            q = q(:);

            q0 = q(1); % Scalar part
            q1 = q(2); % Vector part x
            q2 = q(3); % Vector part y
            q3 = q(4); % Vector part z


            quatDCM = [ 1 - 2 * (q2^2 + q3^2),   2 * (q1 * q2 + q0 * q3),   2 * (q1 * q3 - q0 * q2);
                    2 * (q1 * q2 - q0 * q3),   1 - 2 * (q1^2 + q3^2),   2 * (q2 * q3 + q0 * q1);
                    2 * (q1 * q3 + q0 * q2),   2 * (q2 * q3 - q0 * q1),   1 - 2 * (q1^2 + q2^2) ];

            dcm = quatDCM;
		end

		function dcm = finDcm(deltaDeg, thetaDeg)
			% finDcm: Generates a rotation matrix based on fin location
			% angle (theta) and fin deflection(delta)
			%
    		% INPUT:
    		%   deltaDeg - Fin deflection angle in degrees
			%   thetaDeg - rotation angle along x axis representing fin
			%   location in degrees
			%
    		% OUTPUT:
    		%   dcm - A 3x3 direction cosine matrix (rotation matrix).
    		%
			% Convert the angles from degrees to radians
			delta = deg2rad(deltaDeg);
			theta = deg2rad(thetaDeg);
			
			% Define the first rotation matrix (Rotation about the Y-axis by delta: R_y(delta))
			R_y_delta = [
				cos(delta),  0,          -sin(delta);
				0,           1,          0;
				sin(delta),  0,          cos(delta)
			];
			
			% Define the second rotation matrix (Rotation about the X-axis by theta: R_x(theta))
			R_x_theta = [
				1,           0,          0;
				0,           cos(theta), sin(theta);
				0,           -sin(theta), cos(theta)
			];
			% Compute DCM
			dcm = R_y_delta * R_x_theta;
        end

        function J = numericalJacobian(fun, x0)
            %numericalJacobian Compute the Jacobian matrix of a function at a given point.
            %
            %   J = numericalJacobian(fun, x0)
            %
            %   INPUT:
            %       fun - Function handle, e.g. @(x) [x(1)^2 + x(2); sin(x(1)) + x(2)^3]
            %       x0  - Point (vector) where the Jacobian is evaluated
            %
            %   OUTPUT:
            %       J   - Jacobian matrix (m-by-n), where m = length(fun(x0)), n = length(x0)        
            % Ensure x0 is a column vector
            x0 = x0(:);
            f0 = fun(x0);
            n = length(x0);
            m = length(f0);
            J = zeros(m, n);
        
            % Finite difference step size
            h = sqrt(eps) * max(abs(x0), 1);
        
            for i = 1:n
                x_plus = x0;
                x_minus = x0;
                x_plus(i) = x_plus(i) + h(i);
                x_minus(i) = x_minus(i) - h(i);
        
                f_plus = fun(x_plus);
                f_minus = fun(x_minus);
        
                % Central difference
                J(:, i) = (f_plus - f_minus) / (2 * h(i));
            end
        end
    end
end

