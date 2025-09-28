classdef Utils
    methods (Static)
        % get rotation matrix from a given quaternion
        function vb = quatRotate(q,va)
            % quat_rotate Rotates vec data set using qab from b to a i.e.
            % qBI is Inertial to Body
            % va - 3d vector in b frame (nx3)
            % vb - 3d vec in a frame (nx3)
            % n - # of data entries
            
            dims = size(va);
            
            % normalize quat
            q_norm = (q(:,1).^2 + q(:,2).^2 + q(:,3).^2 + q(:,4).^2) .^ 0.5;
            q_u0 = q(:,1) ./ q_norm;
            q_u1 = q(:,2) ./ q_norm;
            q_u2 = q(:,3) ./ q_norm;
            q_u3 = q(:,4) ./ q_norm;
            q_u = [q_u0 q_u1 q_u2 q_u3];
            
            % quat rotation
            [~, vB1, vB2, vB3] = parts(quaternion(quatconj(q_u)) .* quaternion([zeros(dims(1),1) va]) .* quaternion(q_u));
            vb = [vB1 vB2 vB3];
        end

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

        function dcm = dir_cosine_matlab(q)
            % dir_cosine_matlab: Converts a quaternion to a direction cosine matrix.
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
        
            % Ensure q is a column vector for consistent indexing, though MATLAB handles both
            q = q(:);
        
            % Note: MATLAB uses 1-based indexing, so Python's q[0] is q(1) in MATLAB.
            q0 = q(1); % Scalar part
            q1 = q(2); % Vector part x
            q2 = q(3); % Vector part y
            q3 = q(4); % Vector part z
        
            dcm = [ 1 - 2 * (q2^2 + q3^2),   2 * (q1 * q2 + q0 * q3),   2 * (q1 * q3 - q0 * q2);
                    2 * (q1 * q2 - q0 * q3),   1 - 2 * (q1^2 + q3^2),   2 * (q2 * q3 + q0 * q1);
                    2 * (q1 * q3 + q0 * q2),   2 * (q2 * q3 - q0 * q1),   1 - 2 * (q1^2 + q2^2) ];
        end
    end
end

