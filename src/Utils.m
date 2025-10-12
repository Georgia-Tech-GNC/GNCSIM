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

        function dcm = dir_cosine(q)
            % dir_cosine: Converts a quaternion to a direction cosine matrix.
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
        
            dcm = [ 1 - 2 * (q2^2 + q3^2),   2 * (q1 * q2 + q0 * q3),   2 * (q1 * q3 - q0 * q2);
                    2 * (q1 * q2 - q0 * q3),   1 - 2 * (q1^2 + q3^2),   2 * (q2 * q3 + q0 * q1);
                    2 * (q1 * q3 + q0 * q2),   2 * (q2 * q3 - q0 * q1),   1 - 2 * (q1^2 + q2^2) ];
		end

		function dcm = fin_dcm(delta_deg, theta_deg)
			% fin_dcm: Generates a rotation matrix based on fin location
			% angle (theta) and fin deflection(delta)
			%
    		% INPUT:
    		%   delta_deg - Fin deflection angle in degrees
			%   theta_deg - rotation angle along x axis representing fin
			%   location in degrees
			%
    		% OUTPUT:
    		%   dcm - A 3x3 direction cosine matrix (rotation matrix).
    		%
			% Convert the angles from degrees to radians
			delta = deg2rad(delta_deg);
			theta = deg2rad(theta_deg);
			
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

		function [F_c, M_c] = get_forces_and_moments(state, v_c, aero)
			
			% Aerodynamic coefficients
           	v_a = sqrt(v_c(1)^2+v_c(2)^2+v_c(3)^2);
           	a = 343; % Speed of sound approximation (TODO: Replace with better approximation)
           	M = v_a/a;
           	AoA = atan2d(v_c(3),v_c(1));
	
           	pos = state(2:4);
           	altitude = pos(3);  % z componet of position
           	[ca, cm, cn] = Utils.aero_interp(aero, M, AoA, altitude);
	
           	S = aero.sref;
           	c = aero.cbar;
			% TODO change rho (density) to function of height
			rho = 1.225;
           	q = 0.5 * v_a^2 * rho;  
	
           	% Forces + Moments in component frame
           	N = cn*q*S;
           	A = ca*q*S;
           	M = cm*q*c;
           	F_c = [-A; 0; -N];
		   	M_c = [0; M; 0];
		end

		function [ca, cm, cn] = aero_interp(aero, mach, AoA_deg, alt)
			% TODO change this function to generate functions
            ca_F = griddedInterpolant({aero.alpha(:), aero.mach(:), aero.alt(:)}, ...
                        aero.ca, "linear");
            ca = ca_F(AoA_deg, mach, alt);

            cm_F = griddedInterpolant({aero.alpha(:), aero.mach(:), aero.alt(:)}, ...
                        aero.cm, "linear");
            cm = cm_F(AoA_deg, mach, alt);
            
            cn_F = griddedInterpolant({aero.alpha(:), aero.mach(:), aero.alt(:)}, ...
                        aero.cn, "linear");
			cn = cn_F(AoA_deg, mach, alt);

			alpha_range = [min(aero.alpha(:)), max(aero.alpha(:))];
    		mach_range  = [min(aero.mach(:)), max(aero.mach(:))];
    		alt_range   = [min(aero.alt(:)),  max(aero.alt(:))];
		
    		% Check bounds
    		if AoA_deg < alpha_range(1) || AoA_deg > alpha_range(2)
        		warning('AoA %.2f° is out of range [%.2f, %.2f].', AoA_deg, alpha_range(1), alpha_range(2));
    		end
    		if mach < mach_range(1) || mach > mach_range(2)
        		warning('Mach %.2f is out of range [%.2f, %.2f].', mach, mach_range(1), mach_range(2));
    		end
    		if alt < alt_range(1) || alt > alt_range(2)
        		warning('Altitude %.2f m is out of range [%.2f, %.2f].', alt, alt_range(1), alt_range(2));
    		end
        end
    end
end

