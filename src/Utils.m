% Main Authors: David Reynolds (dreynolds45@gatech.edu)
%               Karsten Caillet (kcaillet3@gatech.edu)

classdef Utils
    methods (Static)

        function nextState = RK4(f, state, dt)
            % Fourth-order Runge-Kutta integration step
            %
            % Inputs:
            %   f     - Function handle that computes the state derivative: xdot = f(x)
            %   state - Current state vector (column vector)
            %   dt    - Time step size
            %
            % Output:
            %   nextState - State vector after one RK4 step

            k1 = f(state);
            k2 = f(state + (k1)*dt/2);
            k3 = f(state + (k2)*dt/2);
            k4 = f(state + (k3)*dt);

            nextState = state + dt*((1/6)*k1 + (1/3)*k2 + (1/3)*k3 + (1/6)*k4);
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
    		
    		% Shorthand for half-angles
    		% a(1)=Roll (phi), a(2)=Pitch (theta), a(3)=Yaw (psi)
    		cR = cos(a(1)*0.5); sR = sin(a(1)*0.5);
    		cP = cos(a(2)*0.5); sP = sin(a(2)*0.5);
    		cY = cos(a(3)*0.5); sY = sin(a(3)*0.5);
		
    		q = zeros(4, 1);
    		% Standard ZYX rotation sequence equations
    		q(1) = cY*cP*cR + sY*sP*sR; % w
    		q(2) = cY*cP*sR - sY*sP*cR; % x
    		q(3) = cY*sP*cR + sY*cP*sR; % y
    		q(4) = sY*cP*cR - cY*sP*sR; % z
        end

		function a = quatToEuler(q)
    		% Convert an N x 4 array of quaternions to Euler angles.
    		%
    		% Input:
    		%   q: An N x 4 array representing quaternions [w, x, y, z].
    		%
    		% Output:
    		%   a: An N x 3 array of Euler angles [roll, pitch, yaw] in degrees.
    		
    		% Ensure q is N x 4 (if user passes 4 x N, transpose it)
    		if size(q, 2) ~= 4 && size(q, 1) == 4
        		q = q';
    		end
    		
    		% Normalize each row (assuming Utils.normalizeQuat handles matrices, 
    		% otherwise use: q = q ./ sqrt(sum(q.^2, 2));)
    		q = Utils.normalizeQuat(q);
    		
    		% Extract columns
    		w = q(:, 1);
    		x = q(:, 2);
    		y = q(:, 3);
    		z = q(:, 4);
    		
    		% Pre-allocate output: N rows, 3 columns
    		numQuats = size(q, 1);
    		a = zeros(numQuats, 3);
    		
    		% Roll (x-axis rotation)
    		sinr_cosp = 2 .* (w .* x + y .* z);
    		cosr_cosp = 1 - 2 .* (x .* x + y .* y);
    		a(:, 1) = atan2(sinr_cosp, cosr_cosp);
    		
    		% Pitch (y-axis rotation)
    		sinp = 2 .* (w .* y - z .* x);
    		% Clamp sinp to [-1, 1] for all rows to avoid complex results from asin
    		sinp = max(-1, min(1, sinp));
    		a(:, 2) = asin(sinp);
    		
    		% Yaw (z-axis rotation)
    		siny_cosp = 2 .* (w .* z + x .* y);
    		cosy_cosp = 1 - 2 .* (y .* y + z .* z);
    		a(:, 3) = atan2(siny_cosp, cosy_cosp);
    		
    		% Convert the entire matrix from radians to degrees
    		a = rad2deg(a);
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
            % from an INERTIAL frame to a BODY frame, based on the input quaternion
            % representing an orientation of a BODY frame with respect
            % to an INERTIAL frame.
            %
            % INPUT:
            %   q - A 4-element column or row vector representing the quaternion [q0; q1; q2; q3],
            %       where q0 is the scalar part.
            %
            % OUTPUT:
            %   dcm - A 3x3 direction cosine matrix (rotation matrix).
            %

            % Ensure q is a column vector
            q = Utils.normalizeQuat(q);
            q = q(:);

            q0 = q(1); % Scalar part
            q1 = q(2); % Vector part x
            q2 = q(3); % Vector part y
            q3 = q(4); % Vector part z
			
			Ry = [0 0 1; 0 1 0; -1 0 0];
            quatDCM = [ 1 - 2 * (q2^2 + q3^2),   2 * (q1 * q2 + q0 * q3),   2 * (q1 * q3 - q0 * q2);
                    2 * (q1 * q2 - q0 * q3),   1 - 2 * (q1^2 + q3^2),   2 * (q2 * q3 + q0 * q1);
                    2 * (q1 * q3 + q0 * q2),   2 * (q2 * q3 - q0 * q1),   1 - 2 * (q1^2 + q2^2) ];

            dcm = Ry * quatDCM;
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
			eps = 1e-6;
        
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

        function quatDot = quatDerivative(quat, omega)
            mat_omega = [
                0, -omega(1), -omega(2), -omega(3);
                omega(1), 0, omega(3), -omega(2);
                omega(2), -omega(3), 0, omega(1);
                omega(3), omega(2), -omega(1), 0
            ];

            quatDot = 0.5 * mat_omega * quat;
        end
    end
end

