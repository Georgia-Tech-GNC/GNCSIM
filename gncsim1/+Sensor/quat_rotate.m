function vB = quat_rotate(q,vI)
            %quat_rotate Rotates vec data set from inertial to body
            % q - quat data set that rotates from I to B - qBI (nx4)
            % vI - 3d vector in I frame (nx3)
            % vB - 3d vec in B frame (nx3)
            % n - # of data entries
            
            dims = size(vI);
            
            % normalize quat
            q_norm = (q(:,1).^2 + q(:,2).^2 + q(:,3).^2 + q(:,4).^2) .^ 0.5;
            q_u0 = q(:,1) ./ q_norm;
            q_u1 = q(:,2) ./ q_norm;
            q_u2 = q(:,3) ./ q_norm;
            q_u3 = q(:,4) ./ q_norm;
            q_u = [q_u0 q_u1 q_u2 q_u3];
            
            % quat rotation
            [~, vB1, vB2, vB3] = parts(quaternion(quatconj(q_u)) .* quaternion([zeros(dims(1),1) vI]) .* quaternion(q_u));
            vB = [vB1 vB2 vB3];
            
            end