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

        
    end
end

