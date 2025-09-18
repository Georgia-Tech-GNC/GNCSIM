classdef Utils
    methods (Static)
        % get rotation matrix from a given quaternion
        function rotation_matrix = get_rotation_matrix_from_quat(q)
            rotation_matrix = [1 - 2 * (q(3)^2 + q(4)^2), 2 * (q(2) * q(3) + q(1) * q(4)), 2 * (q(2) * q(4) - q(1) * q(3));
                                2 * (q(2) * q(3) - q(1) * q(4)), 1 - 2 * (q(2)^2 + q(4)^2), 2 * (q(3) * q(4) + q(1) * q(2));
                                2 * (q(2) * q(4) + q(1) * q(3)), 2 * (q(3) * q(4) - q(1) * q(2)), 1 - 2 * (q(2)^2 + q(3)^2)];
        end

        
    end
end

